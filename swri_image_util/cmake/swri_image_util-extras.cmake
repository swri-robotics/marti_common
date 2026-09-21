# Imports swri_image_util::swri_image_util_display, which is exported
# separately so that it is not part of swri_image_util_TARGETS.  It links
# swri_image_util::swri_image_util, so import that first; ament's own include
# of the same file afterwards is a no-op.
include("${swri_image_util_DIR}/export_swri_image_utilExport.cmake")
include("${swri_image_util_DIR}/export_swri_image_util_display.cmake")
