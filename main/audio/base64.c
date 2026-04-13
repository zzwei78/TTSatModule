#include "audio/base64.h"
#include <string.h>

static const char base64_chars[] =
    "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
    "abcdefghijklmnopqrstuvwxyz"
    "0123456789+/";

static int base64_index(unsigned char c)
{
    if (c >= 'A' && c <= 'Z') return c - 'A';
    if (c >= 'a' && c <= 'z') return c - 'a' + 26;
    if (c >= '0' && c <= '9') return c - '0' + 52;
    if (c == '+') return 62;
    if (c == '/') return 63;
    return -1;
}

int base64_encode(const unsigned char *input, size_t input_len, char *output, size_t output_len)
{
    size_t i, j;
    unsigned char a, b, c;

    // Validate input parameters
    if (output == NULL) {
        return -1;
    }

    // Empty input is valid (produces empty output)
    if (input == NULL || input_len == 0) {
        if (output_len >= 1) {
            output[0] = '\0';
            return 0;
        }
        return -1;
    }

    size_t encoded_len = 4 * ((input_len + 2) / 3);

    // Validate output buffer size (need space for null terminator)
    if (output_len < encoded_len + 1) {
        return -1;
    }

    for (i = 0, j = 0; i < input_len;) {
        a = input[i++];
        b = (i < input_len) ? input[i++] : 0;
        c = (i < input_len) ? input[i++] : 0;

        output[j++] = base64_chars[a >> 2];
        output[j++] = base64_chars[((a & 0x03) << 4) | (b >> 4)];
        output[j++] = base64_chars[((b & 0x0F) << 2) | (c >> 6)];
        output[j++] = base64_chars[c & 0x3F];
    }

    // Add padding
    if (input_len % 3 == 1) {
        output[encoded_len - 2] = '=';
        output[encoded_len - 1] = '=';
    } else if (input_len % 3 == 2) {
        output[encoded_len - 1] = '=';
    }

    output[encoded_len] = '\0';
    return encoded_len;
}

int base64_decode(const char *input, size_t input_len, unsigned char *output, size_t output_len)
{
    size_t i, j;
    int a, b, c, d;
    size_t padding = 0;

    // Validate input parameters
    if (input == NULL || output == NULL) {
        return -1;
    }

    // Empty input is valid (produces empty output)
    if (input_len == 0) {
        return 0;
    }

    // Base64 encoded length must be multiple of 4
    if (input_len % 4 != 0) {
        return -1;
    }

    // Calculate decoded length (must check input_len >= 4 before accessing padding)
    size_t decoded_len = 3 * input_len / 4;

    // Check for padding characters (safe now because input_len >= 4)
    if (input_len >= 1 && input[input_len - 1] == '=') padding++;
    if (input_len >= 2 && input[input_len - 2] == '=') padding++;

    decoded_len -= padding;

    // Validate output buffer size
    if (output_len < decoded_len) {
        return -1;
    }

    for (i = 0, j = 0; i < input_len;) {
        a = base64_index(input[i++]);
        b = base64_index(input[i++]);
        c = base64_index(input[i++]);
        d = base64_index(input[i++]);

        // Validate indices (except for padding positions)
        if (a < 0 || b < 0) {
            return -1;
        }

        output[j++] = (a << 2) | (b >> 4);

        if (c >= 0) {
            output[j++] = (b << 4) | (c >> 2);

            if (d >= 0) {
                output[j++] = (c << 6) | d;
            }
        }
    }

    return decoded_len;
}
