#define _POSIX_C_SOURCE 200809L

#include <arpa/inet.h>
#include <errno.h>
#include <limits.h>
#include <netdb.h>
#include <netinet/in.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

#include <elog.h>

#include "jsonrpcServer.h"

#define JRPC_MAX_REQUEST_SIZE (1024U * 1024U)
#define JRPC_MAX_HEADER_SIZE  (16U * 1024U)

static void close_connection(struct ev_loop *loop, jrpc_connection *connection)
{
    if (connection == NULL) {
        return;
    }

    ev_io_stop(loop, &connection->io);
    if (connection->fd >= 0) {
        close(connection->fd);
    }
    free(connection->buffer);
    free(connection);
}

static cJSON *copy_json_value(const cJSON *value)
{
    char *text;
    cJSON *copy;

    if (value == NULL) {
        return cJSON_CreateNull();
    }

    text = cJSON_PrintUnformatted((cJSON *)value);
    if (text == NULL) {
        return NULL;
    }
    copy = cJSON_Parse(text);
    free(text);
    return copy;
}

static cJSON *create_error_response(int code, const char *message,
                                    const cJSON *id)
{
    cJSON *response = cJSON_CreateObject();
    cJSON *error = cJSON_CreateObject();
    cJSON *id_copy = copy_json_value(id);

    if (response == NULL || error == NULL || id_copy == NULL) {
        cJSON_Delete(response);
        cJSON_Delete(error);
        cJSON_Delete(id_copy);
        return NULL;
    }

    cJSON_AddStringToObject(response, "jsonrpc", "2.0");
    cJSON_AddNumberToObject(error, "code", code);
    cJSON_AddStringToObject(error, "message",
                            message != NULL ? message : "Internal error");
    cJSON_AddItemToObject(response, "error", error);
    cJSON_AddItemToObject(response, "id", id_copy);
    return response;
}

static cJSON *create_result_response(cJSON *result, const cJSON *id)
{
    cJSON *response = cJSON_CreateObject();
    cJSON *id_copy = copy_json_value(id);

    if (result == NULL) {
        result = cJSON_CreateNull();
    }
    if (response == NULL || result == NULL || id_copy == NULL) {
        cJSON_Delete(response);
        cJSON_Delete(result);
        cJSON_Delete(id_copy);
        return NULL;
    }

    cJSON_AddStringToObject(response, "jsonrpc", "2.0");
    cJSON_AddItemToObject(response, "result", result);
    cJSON_AddItemToObject(response, "id", id_copy);
    return response;
}

static jrpc_procedure *find_procedure(jrpc_server *server, const char *name)
{
    int i;

    for (i = 0; i < server->procedure_count; ++i) {
        if (strcmp(server->procedures[i].name, name) == 0) {
            return &server->procedures[i];
        }
    }
    return NULL;
}

static int is_valid_id(const cJSON *id)
{
    int type;

    if (id == NULL) {
        return 1;
    }
    type = id->type & 0xff;
    return type == cJSON_String || type == cJSON_Number || type == cJSON_NULL;
}

/*
 * 处理一个JSON-RPC请求。没有id的请求是通知：仍执行回调，但不返回响应。
 */
static cJSON *dispatch_request(jrpc_server *server, cJSON *request)
{
    cJSON *version;
    cJSON *method;
    cJSON *params;
    cJSON *id;
    cJSON *result;
    jrpc_procedure *procedure;
    jrpc_context context;
    int notification;

    if (request == NULL || (request->type & 0xff) != cJSON_Object) {
        return create_error_response(JRPC_INVALID_REQUEST,
                                     "Invalid Request", NULL);
    }

    version = cJSON_GetObjectItem(request, "jsonrpc");
    method = cJSON_GetObjectItem(request, "method");
    params = cJSON_GetObjectItem(request, "params");
    id = cJSON_GetObjectItem(request, "id");
    notification = (id == NULL);

    if (version == NULL ||
        (version->type & 0xff) != cJSON_String ||
        version->valuestring == NULL ||
        strcmp(version->valuestring, "2.0") != 0 ||
        method == NULL || (method->type & 0xff) != cJSON_String ||
        method->valuestring == NULL || method->valuestring[0] == '\0' ||
        (params != NULL && (params->type & 0xff) != cJSON_Object &&
         (params->type & 0xff) != cJSON_Array) ||
        !is_valid_id(id)) {
        return create_error_response(JRPC_INVALID_REQUEST,
                                     "Invalid Request", NULL);
    }

    procedure = find_procedure(server, method->valuestring);
    if (procedure == NULL) {
        if (notification) {
            return NULL;
        }
        return create_error_response(JRPC_METHOD_NOT_FOUND,
                                     "Method not found", id);
    }

    memset(&context, 0, sizeof(context));
    context.data = procedure->data;
    result = procedure->function(&context, params, id);

    if (notification) {
        cJSON_Delete(result);
        free(context.error_message);
        return NULL;
    }

    if (context.error_code != 0) {
        cJSON *error = create_error_response(
            context.error_code,
            context.error_message != NULL ? context.error_message
                                          : "Internal error",
            id);
        cJSON_Delete(result);
        free(context.error_message);
        return error;
    }

    free(context.error_message);
    return create_result_response(result, id);
}

static cJSON *dispatch_document(jrpc_server *server, cJSON *document)
{
    cJSON *responses;
    cJSON *request;

    if (document == NULL) {
        return create_error_response(JRPC_PARSE_ERROR, "Parse error", NULL);
    }
    if ((document->type & 0xff) != cJSON_Array) {
        return dispatch_request(server, document);
    }
    if (cJSON_GetArraySize(document) == 0) {
        return create_error_response(JRPC_INVALID_REQUEST,
                                     "Invalid Request", NULL);
    }

    responses = cJSON_CreateArray();
    if (responses == NULL) {
        return create_error_response(JRPC_INTERNAL_ERROR,
                                     "Internal error", NULL);
    }

    request = document->child;
    while (request != NULL) {
        cJSON *response = dispatch_request(server, request);
        if (response != NULL) {
            cJSON_AddItemToArray(responses, response);
        }
        request = request->next;
    }

    if (cJSON_GetArraySize(responses) == 0) {
        cJSON_Delete(responses);
        return NULL;
    }
    return responses;
}

static int send_all(int fd, const char *data, size_t length)
{
    size_t sent = 0;

    while (sent < length) {
        ssize_t result;
#ifdef MSG_NOSIGNAL
        result = send(fd, data + sent, length - sent, MSG_NOSIGNAL);
#else
        result = send(fd, data + sent, length - sent, 0);
#endif
        if (result > 0) {
            sent += (size_t)result;
            continue;
        }
        if (result < 0 && errno == EINTR) {
            continue;
        }
        return -1;
    }
    return 0;
}

static int send_response(jrpc_connection *connection, cJSON *response,
                         int use_http)
{
    char *body;
    size_t body_length;
    int status = 0;

    if (response == NULL) {
        return 0;
    }
    body = cJSON_PrintUnformatted(response);
    if (body == NULL) {
        return -1;
    }
    body_length = strlen(body);

    if (use_http) {
        char header[256];
        int header_length = snprintf(
            header, sizeof(header),
            "HTTP/1.1 200 OK\r\n"
            "Content-Type: application/json; charset=UTF-8\r\n"
            "Content-Length: %zu\r\n"
            "Connection: keep-alive\r\n\r\n",
            body_length);

        if (header_length < 0 || (size_t)header_length >= sizeof(header) ||
            send_all(connection->fd, header, (size_t)header_length) < 0 ||
            send_all(connection->fd, body, body_length) < 0) {
            status = -1;
        }
    } else if (send_all(connection->fd, body, body_length) < 0 ||
               send_all(connection->fd, "\n", 1) < 0) {
        status = -1;
    }

    free(body);
    return status;
}

static int parse_and_respond(jrpc_server *server,
                             jrpc_connection *connection,
                             const char *text, int use_http)
{
    char *end = NULL;
    cJSON *document = cJSON_Parse_Stream(text, &end);
    cJSON *response;
    int status;

    if (document == NULL) {
        response = create_error_response(JRPC_PARSE_ERROR,
                                         "Parse error", NULL);
    } else {
        while (end != NULL && (*end == ' ' || *end == '\t' ||
                               *end == '\r' || *end == '\n')) {
            ++end;
        }
        if (end == NULL || *end != '\0') {
            cJSON_Delete(document);
            document = NULL;
            response = create_error_response(JRPC_PARSE_ERROR,
                                             "Parse error", NULL);
        } else {
            response = dispatch_document(server, document);
        }
    }

    status = send_response(connection, response, use_http);
    cJSON_Delete(response);
    cJSON_Delete(document);
    return status;
}

static const char *find_header_end(const char *buffer)
{
    return strstr(buffer, "\r\n\r\n");
}

/* 返回0表示成功，-1表示HTTP请求头不合法。 */
static int parse_http_header(const char *header, size_t header_length,
                             size_t *content_length)
{
    const char *line;
    const char *header_end = header + header_length;
    int found_length = 0;

    if (header_length < 5 || strncmp(header, "POST ", 5) != 0) {
        return -1;
    }

    line = strstr(header, "\r\n");
    if (line == NULL || line + 2 > header_end) {
        return -1;
    }
    line += 2;

    while (line < header_end - 2) {
        const char *line_end = strstr(line, "\r\n");
        const char name[] = "Content-Length:";

        if (line_end == NULL || line_end > header_end) {
            return -1;
        }
        if ((size_t)(line_end - line) >= sizeof(name) - 1 &&
            strncasecmp(line, name, sizeof(name) - 1) == 0) {
            const char *value = line + sizeof(name) - 1;
            char *number_end;
            unsigned long long parsed;

            if (found_length) {
                return -1;
            }
            while (value < line_end && (*value == ' ' || *value == '\t')) {
                ++value;
            }
            if (value == line_end || *value == '-') {
                return -1;
            }
            errno = 0;
            parsed = strtoull(value, &number_end, 10);
            while (number_end < line_end &&
                   (*number_end == ' ' || *number_end == '\t')) {
                ++number_end;
            }
            if (errno == ERANGE || number_end != line_end ||
                parsed > JRPC_MAX_REQUEST_SIZE) {
                return -1;
            }
            *content_length = (size_t)parsed;
            found_length = 1;
        }
        line = line_end + 2;
    }

    return found_length ? 0 : -1;
}

static int process_http_requests(jrpc_server *server,
                                 jrpc_connection *connection)
{
    for (;;) {
        const char *separator = find_header_end(connection->buffer);
        size_t header_length;
        size_t body_length;
        size_t request_length;
        char *body;

        if (separator == NULL) {
            return connection->pos > JRPC_MAX_HEADER_SIZE ? -1 : 0;
        }
        header_length = (size_t)(separator - connection->buffer) + 4U;
        if (header_length > JRPC_MAX_HEADER_SIZE ||
            parse_http_header(connection->buffer, header_length,
                              &body_length) < 0 ||
            header_length > SIZE_MAX - body_length) {
            return -1;
        }
        request_length = header_length + body_length;
        if (connection->pos < request_length) {
            return 0;
        }

        body = malloc(body_length + 1U);
        if (body == NULL) {
            return -1;
        }
        memcpy(body, connection->buffer + header_length, body_length);
        body[body_length] = '\0';
        if (parse_and_respond(server, connection, body, 1) < 0) {
            free(body);
            return -1;
        }
        free(body);

        connection->pos -= (unsigned int)request_length;
        memmove(connection->buffer,
                connection->buffer + request_length,
                connection->pos);
        connection->buffer[connection->pos] = '\0';
        if (connection->pos == 0) {
            return 0;
        }
    }
}

static int grow_connection_buffer(jrpc_connection *connection)
{
    unsigned int new_size;
    char *new_buffer;

    if (connection->buffer_size >= JRPC_MAX_REQUEST_SIZE +
                                       JRPC_MAX_HEADER_SIZE) {
        return -1;
    }
    new_size = connection->buffer_size * 2U;
    if (new_size > JRPC_MAX_REQUEST_SIZE + JRPC_MAX_HEADER_SIZE) {
        new_size = JRPC_MAX_REQUEST_SIZE + JRPC_MAX_HEADER_SIZE;
    }
    new_buffer = realloc(connection->buffer, (size_t)new_size + 1U);
    if (new_buffer == NULL) {
        return -1;
    }
    connection->buffer = new_buffer;
    connection->buffer_size = new_size;
    return 0;
}

static void connection_cb(struct ev_loop *loop, ev_io *watcher, int events)
{
    jrpc_connection *connection = (jrpc_connection *)watcher;
    jrpc_server *server = watcher->data;
    ssize_t bytes_read;
    int status;

    if ((events & EV_ERROR) != 0) {
        close_connection(loop, connection);
        return;
    }
    if (connection->pos == connection->buffer_size &&
        grow_connection_buffer(connection) < 0) {
        close_connection(loop, connection);
        return;
    }

    bytes_read = recv(connection->fd,
                      connection->buffer + connection->pos,
                      connection->buffer_size - connection->pos, 0);
    if (bytes_read < 0 && (errno == EINTR || errno == EAGAIN ||
                           errno == EWOULDBLOCK)) {
        return;
    }
    if (bytes_read <= 0) {
        close_connection(loop, connection);
        return;
    }

    connection->pos += (unsigned int)bytes_read;
    connection->buffer[connection->pos] = '\0';
    if (server->type == JRPC_TYPE_HTTP) {
        status = process_http_requests(server, connection);
    } else {
        status = parse_and_respond(server, connection,
                                   connection->buffer, 0);
        connection->pos = 0;
        connection->buffer[0] = '\0';
    }
    if (status < 0) {
        close_connection(loop, connection);
    }
}

static void accept_cb(struct ev_loop *loop, ev_io *watcher, int events)
{
    jrpc_server *server = watcher->data;
    jrpc_connection *connection;
    int fd;

    if ((events & EV_ERROR) != 0) {
        log_e("JSON-RPC监听器发生错误");
        return;
    }
    fd = accept(watcher->fd, NULL, NULL);
    if (fd < 0) {
        if (errno != EINTR && errno != EAGAIN && errno != EWOULDBLOCK) {
            log_e("接受JSON-RPC连接失败: %s", strerror(errno));
        }
        return;
    }

    connection = calloc(1, sizeof(*connection));
    if (connection == NULL) {
        close(fd);
        return;
    }
    connection->buffer = malloc(JRPC_BUFFER_SIZE + 1U);
    if (connection->buffer == NULL) {
        close(fd);
        free(connection);
        return;
    }
    connection->fd = fd;
    connection->buffer_size = JRPC_BUFFER_SIZE;
    connection->debug_level = server->debug_level;
    connection->buffer[0] = '\0';
    ev_io_init(&connection->io, connection_cb, fd, EV_READ);
    connection->io.data = server;
    ev_io_start(loop, &connection->io);
}

static int start_server(jrpc_server *server)
{
    struct addrinfo hints;
    struct addrinfo *addresses = NULL;
    struct addrinfo *address;
    char port[16];
    int listener = -1;
    int reuse = 1;
    int result;

    memset(&hints, 0, sizeof(hints));
    hints.ai_family = AF_UNSPEC;
    hints.ai_socktype = SOCK_STREAM;
    hints.ai_flags = AI_PASSIVE;
    snprintf(port, sizeof(port), "%d", server->port_number);

    result = getaddrinfo(NULL, port, &hints, &addresses);
    if (result != 0) {
        log_e("解析JSON-RPC监听地址失败: %s", gai_strerror(result));
        return -1;
    }

    for (address = addresses; address != NULL; address = address->ai_next) {
        listener = socket(address->ai_family, address->ai_socktype,
                          address->ai_protocol);
        if (listener < 0) {
            continue;
        }
        if (setsockopt(listener, SOL_SOCKET, SO_REUSEADDR,
                       &reuse, sizeof(reuse)) == 0 &&
            bind(listener, address->ai_addr, address->ai_addrlen) == 0 &&
            listen(listener, 16) == 0) {
            break;
        }
        close(listener);
        listener = -1;
    }
    freeaddrinfo(addresses);

    if (listener < 0) {
        log_e("启动JSON-RPC监听端口%d失败: %s",
              server->port_number, strerror(errno));
        return -1;
    }

    ev_io_init(&server->listen_watcher, accept_cb, listener, EV_READ);
    server->listen_watcher.data = server;
    ev_io_start(server->loop, &server->listen_watcher);
    return 0;
}

int jrpc_server_init(jrpc_server *server, int port_number)
{
    return jrpc_server_init_with_ev_loop(server, port_number, EV_DEFAULT);
}

int jrpc_server_init_with_ev_loop(jrpc_server *server, int port_number,
                                  struct ev_loop *loop)
{
    if (server == NULL || loop == NULL || port_number < 0 ||
        port_number > 65535) {
        return -1;
    }

    memset(server, 0, sizeof(*server));
    server->port_number = port_number;
    server->loop = loop;
    server->type = JRPC_TYPE_NORMAL;
    server->listen_watcher.fd = -1;
    return start_server(server);
}

void jrpc_server_run(jrpc_server *server)
{
    if (server != NULL && server->loop != NULL) {
        ev_run(server->loop, 0);
    }
}

int jrpc_server_stop(jrpc_server *server)
{
    if (server == NULL || server->loop == NULL) {
        return -1;
    }
    if (server->listen_watcher.fd >= 0) {
        ev_io_stop(server->loop, &server->listen_watcher);
        close(server->listen_watcher.fd);
        server->listen_watcher.fd = -1;
    }
    ev_break(server->loop, EVBREAK_ALL);
    return 0;
}

void jrpc_server_destroy(jrpc_server *server)
{
    int i;

    if (server == NULL) {
        return;
    }
    if (server->listen_watcher.fd >= 0) {
        jrpc_server_stop(server);
    }
    for (i = 0; i < server->procedure_count; ++i) {
        free(server->procedures[i].name);
    }
    free(server->procedures);
    server->procedures = NULL;
    server->procedure_count = 0;
}

int jrpc_register_procedure(jrpc_server *server, jrpc_function function,
                            char *name, void *data)
{
    jrpc_procedure *procedures;
    char *name_copy;

    if (server == NULL || function == NULL || name == NULL || name[0] == '\0' ||
        find_procedure(server, name) != NULL) {
        return -1;
    }

    name_copy = strdup(name);
    if (name_copy == NULL) {
        return -1;
    }
    procedures = realloc(server->procedures,
                         (size_t)(server->procedure_count + 1) *
                             sizeof(*procedures));
    if (procedures == NULL) {
        free(name_copy);
        return -1;
    }

    server->procedures = procedures;
    server->procedures[server->procedure_count].name = name_copy;
    server->procedures[server->procedure_count].function = function;
    server->procedures[server->procedure_count].data = data;
    ++server->procedure_count;
    return 0;
}
