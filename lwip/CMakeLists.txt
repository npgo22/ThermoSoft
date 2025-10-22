# lwIP middleware sources
add_library(lwip_middleware OBJECT)

target_sources(lwip_middleware PRIVATE
    # lwIP Core sources
    ${CMAKE_SOURCE_DIR}/src/core/init.c
    ${CMAKE_SOURCE_DIR}/src/core/def.c
    ${CMAKE_SOURCE_DIR}/src/core/dns.c
    ${CMAKE_SOURCE_DIR}/src/core/inet_chksum.c
    ${CMAKE_SOURCE_DIR}/src/core/ip.c
    ${CMAKE_SOURCE_DIR}/src/core/mem.c
    ${CMAKE_SOURCE_DIR}/src/core/memp.c
    ${CMAKE_SOURCE_DIR}/src/core/netif.c
    ${CMAKE_SOURCE_DIR}/src/core/pbuf.c
    ${CMAKE_SOURCE_DIR}/src/core/raw.c
    ${CMAKE_SOURCE_DIR}/src/core/stats.c
    ${CMAKE_SOURCE_DIR}/src/core/sys.c
    ${CMAKE_SOURCE_DIR}/src/core/altcp.c
    ${CMAKE_SOURCE_DIR}/src/core/altcp_alloc.c
    ${CMAKE_SOURCE_DIR}/src/core/altcp_tcp.c
    ${CMAKE_SOURCE_DIR}/src/core/tcp.c
    ${CMAKE_SOURCE_DIR}/src/core/tcp_in.c
    ${CMAKE_SOURCE_DIR}/src/core/tcp_out.c
    ${CMAKE_SOURCE_DIR}/src/core/timeouts.c
    ${CMAKE_SOURCE_DIR}/src/core/udp.c
    
    # lwIP IPv4 sources
    ${CMAKE_SOURCE_DIR}/src/core/ipv4/autoip.c
    ${CMAKE_SOURCE_DIR}/src/core/ipv4/dhcp.c
    ${CMAKE_SOURCE_DIR}/src/core/ipv4/etharp.c
    ${CMAKE_SOURCE_DIR}/src/core/ipv4/icmp.c
    ${CMAKE_SOURCE_DIR}/src/core/ipv4/igmp.c
    ${CMAKE_SOURCE_DIR}/src/core/ipv4/ip4_frag.c
    ${CMAKE_SOURCE_DIR}/src/core/ipv4/ip4.c
    ${CMAKE_SOURCE_DIR}/src/core/ipv4/ip4_addr.c
    ${CMAKE_SOURCE_DIR}/src/core/ipv4/acd.c
    
    # lwIP API sources
    ${CMAKE_SOURCE_DIR}/src/api/api_lib.c
    ${CMAKE_SOURCE_DIR}/src/api/api_msg.c
    ${CMAKE_SOURCE_DIR}/src/api/err.c
    ${CMAKE_SOURCE_DIR}/src/api/if_api.c
    ${CMAKE_SOURCE_DIR}/src/api/netbuf.c
    ${CMAKE_SOURCE_DIR}/src/api/netdb.c
    ${CMAKE_SOURCE_DIR}/src/api/netifapi.c
    ${CMAKE_SOURCE_DIR}/src/api/sockets.c
    ${CMAKE_SOURCE_DIR}/src/api/tcpip.c
    
    # lwIP netif sources
    ${CMAKE_SOURCE_DIR}/src/netif/ethernet.c
    ${CMAKE_SOURCE_DIR}/src/netif/bridgeif.c
    ${CMAKE_SOURCE_DIR}/src/netif/bridgeif_fdb.c
    ${CMAKE_SOURCE_DIR}/src/netif/slipif.c
    
    # Application sources
    ${CMAKE_SOURCE_DIR}/lwip/app/lwip.c
    ${CMAKE_SOURCE_DIR}/lwip/target/ethernetif.c
    
    # BSP drivers
    ${CMAKE_SOURCE_DIR}/Drivers/BSP/lan8742.c
    
    # System/OS sources
    ${CMAKE_SOURCE_DIR}/system/OS/sys_arch.c
)

target_include_directories(lwip_middleware PUBLIC
    ${CMAKE_SOURCE_DIR}/lwip/app
    ${CMAKE_SOURCE_DIR}/lwip/target
    ${CMAKE_SOURCE_DIR}/src/include
    ${CMAKE_SOURCE_DIR}/system
    ${CMAKE_SOURCE_DIR}/Drivers/BSP
    ${CMAKE_SOURCE_DIR}/Core/Inc
    ${CMAKE_SOURCE_DIR}/Drivers/STM32H5xx_HAL_Driver/Inc
    ${CMAKE_SOURCE_DIR}/Drivers/STM32H5xx_HAL_Driver/Inc/Legacy
    ${CMAKE_SOURCE_DIR}/Drivers/CMSIS/Device/ST/STM32H5xx/Include
    ${CMAKE_SOURCE_DIR}/Drivers/CMSIS/Include
)

target_compile_definitions(lwip_middleware PUBLIC
    LWIP_TIMEVAL_PRIVATE=0
    STM32H563xx
    USE_HAL_DRIVER
)
