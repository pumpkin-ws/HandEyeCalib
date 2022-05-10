/**
 * @brief utility to log error and debug information to console
 * 
 */
#define LOG_ERROR(...) do {\
    printf("<%s %s> %s %s:%d \033[47;31mERROR: ",__DATE__, __TIME__,__FUNCTION__,__FILE__,__LINE__)&&\
    printf(__VA_ARGS__)&&\
    printf("\033[0m\n");\
}while(0)

#define LOG_WARNING(...) do {\
    printf("<%s %s> %s %s:%d \033[1;33mWARNING: ",__DATE__, __TIME__,__FUNCTION__,__FILE__,__LINE__)&&\
    printf(__VA_ARGS__)&&\
    printf("\033[0m\n");\
}while(0)

#define LOG_DEBUG(...) do {\
    printf("<%s %s> %s %s:%d \033[0;34mDEBUG: ",__DATE__, __TIME__,__FUNCTION__,__FILE__,__LINE__)&&\
    printf(__VA_ARGS__)&&\
    printf("\033[0m\n");\
}while(0)

#define LOG_RED(...) do {\
    printf("\033[1;31mERROR: ")&&\
    printf(__VA_ARGS__)&&\
    printf("\033[0m\n");\
}while(0)

#define LOG_GREEN(...) do {\
    printf("\033[1;32mINFO: ")&&\
    printf(__VA_ARGS__)&&\
    printf("\033[0m\n");\
}while(0)

#define LOG_BLUE(...) do {\
    printf("\033[1;34mWARNING: ")&&\
    printf(__VA_ARGS__)&&\
    printf("\033[0m\n");\
}while(0)