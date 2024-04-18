// #include "cli.hpp"
// #include <cstring>

// extern const struct cli::cli_handler_t __cli_start;
// extern const struct cli::cli_handler_t __cli_stop;

// bool cli::cliHandler(char string[])
// {
//     uint8_t argc = 0;
//     char *argv[maxArgCount];

//     argv[0] = strtok(string, " ");
//     do
//     {
//         for (char *ptr = argv[argc]; *ptr; ++ptr)
//             if (*ptr == '\r' or *ptr == '\n')
//             {
//                 *ptr = '\0';
//                 break;
//             }
//             else if (*ptr <= 0x7Fu)
//                 *ptr = (*ptr);
//             else if (*ptr == 0xA8u) // Ё
//                 *ptr = 0xB8u;
//             else if (*ptr >= 0xC0u and *ptr <= 0xDFu)
//                 *ptr += 0x20u;

//         ++argc;
//     } while (argc < sizeof(argv) / sizeof(argv[0]) and (argv[argc] = strtok(nullptr, " ")));

//     unsigned L = 0;
//     unsigned R = (&__cli_stop - &__cli_start) - 2;
//     while (L <= R)
//     {
//         unsigned m = (L + R) / 2;
//         auto i = strcmp(((&__cli_start)[m]).alias, argv[0]);
//         if (i < 0)
//             L = m + 1;
//         else if (i > 0)
//             R = m - 1;
//         else
//         {
//             (&__cli_start)[m].callback(argc, argv);
//             return true;
//         }
//     }

//     // defaultCliHandler(argc, argv);
//     return false;
// }
