idf_component_register(SRCS "main.c" "cobs.c" "bus.c" "websrv.c"
                    INCLUDE_DIRS "inc"
                    EMBED_TXTFILES web/ota.html web/nodefw.html web/setaddr.html web/editnodes.html web/nodes.html web/network.html web/cal.html)
