idf_component_register(
    SRCS "pr_protocol_legacy.c" "pr_config.c"
    INCLUDE_DIRS "include"
    REQUIRES nvs_flash
)
