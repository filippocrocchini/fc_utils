/*Author:
    -Filippo Crocchini

    Copyright :
    This library is available under the MIT license, see end of the file.

    Usage :
        // Do this in only one file

    #define FC_URI_PARSE_IMPLEMENTATION
    #include "fc_uri_parse.h"

    // Just include as usual in the others
    #include "fc_uri_parse.h"

Example:
    fc_uri repository_uri;
    fc_uri_parse("https://github.com/filippocrocchini/fc_utils", &repository_uri);

    // use repository_uri

Info:
    Parser for URIs according to RFC3986 (https://www.rfc-editor.org/rfc/rfc3986)
    It does NOT implement relative URIs
*/

#ifndef FC_URI_PARSE
#define FC_URI_PARSE

#define FC_URI_MAX 2083

#ifndef __cplusplus
#include <stdbool.h>
#endif

typedef struct
{
    char* scheme;

    // Authority
    char* user;
    char* access_info;
    char* host;
    char* port;

    char* path;
    char* query;
    char* fragment;

    bool ipv6_host;

    char buf[FC_URI_MAX + 8]; // Eight '\0' to null-terminate all possible fields. Some characters are ignored so, this is probably too many.
} fc_uri;

void fc_uri_parse(const char* src, fc_uri* uri);

#ifdef FC_URI_PARSE_IMPLEMENTATION

typedef struct 
{
    char* data;
    int  count;
} fc_str;

typedef struct
{
    char* in_buffer;
    char* in_buffer_end;

    char* cursor;
} fc_parser_state;

bool fc_accept_char(fc_parser_state* state, char c)
{
    if (state->cursor != state->in_buffer_end && *state->cursor == c)
    {
        state->cursor++;
        return true;
    }
    return false;
}

bool fc_accept_str(fc_parser_state* state, const char* str)
{
    char* c = state->cursor;
    while (c != state->in_buffer_end && *str && *c++ == *str++);
    
    if (*str == '\0')
    {
        state->cursor = c;
        return true;
    }

    return false;
}

fc_str fc_read_until(fc_parser_state* state, const char* stop_at)
{
    fc_str result = {0};

    result.data  = state->cursor;

    bool not_found = true;
    while (state->cursor != state->in_buffer_end && *state->cursor && not_found)
    {
        for (const char* s = stop_at; *state->cursor && *s; ++s)
        {
            if (*state->cursor == *s)
            {
                not_found = false;
                break;
            }
        }

        if (not_found) {
            state->cursor += 1;
        }
    }

    result.count = state->cursor - result.data;
    
    return result;
}

char* fc_copy_string(const char* str, int count, char* target_buffer, int* buffer_offest)
{
    if (!count) return NULL;

    char* result = target_buffer + *buffer_offest;

    for (int i = 0; i < count; ++i)
    {
        *(target_buffer + (*buffer_offest)++) = *str++;
    }

    *(target_buffer + (*buffer_offest)++) = '\0';

    return result;
}

void fc_init_parser_state(fc_parser_state* state, const char* input, int input_size = -1)
{
    state->in_buffer = (char*)input;
    state->in_buffer_end = input_size > 0 ? (char*)input + input_size : NULL;
    state->cursor = state->in_buffer;
}

void fc_uri_parse(const char* src, fc_uri* uri)
{
    fc_parser_state parser = {};

    fc_init_parser_state(&parser, src);

    uri->ipv6_host = false;

    fc_str scheme = fc_read_until(&parser, ":");

    fc_str authority = {};
    fc_str user = {};
    fc_str access_info = {};
    fc_str host = {};
    fc_str port = {};

    fc_str path = {};
    fc_str query = {};
    fc_str fragment = {};

    if (!fc_accept_char(&parser, ':'))
    {
        // No scheme, likely a relative resource. Which we don't support. (yet?)
        return;
    }

    if (fc_accept_str(&parser, "//"))
    {
        authority = fc_read_until(&parser, "/");

        fc_parser_state authority_parser = {};
        fc_init_parser_state(&authority_parser, authority.data, authority.count);

        if (fc_accept_char(&authority_parser, '['))
        {
            uri->ipv6_host = true;
            // [host]:port/
            host = fc_read_until(&authority_parser, "]");
            if (!fc_accept_char(&authority_parser, ']'))
            {
                // Missing closed bracket
                return;
            }

            if (fc_accept_char(&authority_parser, ':'))
            {
                port = fc_read_until(&authority_parser, "/");
            }
        }
        else {
            // host:port/
            // user:pwd@[host]:port/

            fc_str host_ot_user = fc_read_until(&authority_parser, ":/");
            
            fc_accept_char(&authority_parser, ':');
            
            fc_str access_info_or_port = fc_read_until(&authority_parser, "@/");

            if (fc_accept_char(&authority_parser, '@'))
            {
                user = host_ot_user;
                access_info = access_info_or_port;

                // @[host]:port
                // @host:port

                if (fc_accept_char(&authority_parser, '['))
                {
                    uri->ipv6_host = true;

                    host = fc_read_until(&authority_parser, "]");
                    if (!fc_accept_char(&authority_parser, ']'))
                    {
                        // Missing closed bracket
                        return;
                    }
                }
                else {
                    host = fc_read_until(&authority_parser, ":/");
                }

                if (fc_accept_char(&authority_parser, ':'))
                {
                    port = fc_read_until(&authority_parser, "/");
                }
            }
            else {
                host = host_ot_user;
                port = access_info_or_port;
            }
        }

    }

    path = fc_read_until(&parser, "?");

    if (fc_accept_char(&parser, '?'))
    {
        query = fc_read_until(&parser, "#");
    }

    if (fc_accept_char(&parser, '#'))
    {
        fragment = fc_read_until(&parser, "");
    }

    int offset = 0;

    uri->scheme   = fc_copy_string(scheme.data,    scheme.count,    uri->buf, &offset);
    uri->user     = fc_copy_string(user.data,      user.count,      uri->buf, &offset);
    uri->access_info = fc_copy_string(access_info.data, access_info.count, uri->buf, &offset);
    uri->host     = fc_copy_string(host.data,      host.count,      uri->buf, &offset);
    uri->port     = fc_copy_string(port.data,      port.count,      uri->buf, &offset);
    uri->path     = fc_copy_string(path.data,      path.count,      uri->buf, &offset);
    uri->query    = fc_copy_string(query.data,     query.count,     uri->buf, &offset);
    uri->fragment = fc_copy_string(fragment.data,  fragment.count,  uri->buf, &offset);
}

#endif // FC_URI_PARSE_IMPLEMENTATION

#endif

/*
    Copyright (c) 2022 Filippo Crocchini

    Permission is hereby granted, free of charge, to any person obtaining a copy
    of this software and associated documentation files (the "Software"), to deal
    in the Software without restriction, including without limitation the rights
    to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
    copies of the Software, and to permit persons to whom the Software is
    furnished to do so, subject to the following conditions:

    The above copyright notice and this permission notice shall be included in all
    copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
    AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
    OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
    SOFTWARE.
*/