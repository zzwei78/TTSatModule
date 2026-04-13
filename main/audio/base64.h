#ifndef __BASE64_H__
#define __BASE64_H__

#include <stddef.h>

/**
 * @brief Base64 encoding function
 * @param input Input binary data
 * @param input_len Length of input data
 * @param output Output buffer for Base64 encoded string
 * @param output_len Size of output buffer
 * @return Returns encoded string length on success, -1 on failure
 */
int base64_encode(const unsigned char *input, size_t input_len, char *output, size_t output_len);

/**
 * @brief Base64 decoding function
 * @param input Base64 encoded string
 * @param input_len Length of input string
 * @param output Output buffer for binary data
 * @param output_len Size of output buffer
 * @return Returns decoded binary data length on success, -1 on failure
 */
int base64_decode(const char *input, size_t input_len, unsigned char *output, size_t output_len);

#endif /* __BASE64_H__ */
