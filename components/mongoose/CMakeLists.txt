idf_component_register(
    SRCS "mongoose.c" "mongoose_impl.c" "mongoose_fs.c" "mongoose_glue.c"
    INCLUDE_DIRS 
        "include"
    REQUIRES 
        esp_wifi 
        esp_event 
        esp_netif 
        app_update
        esp_timer
)