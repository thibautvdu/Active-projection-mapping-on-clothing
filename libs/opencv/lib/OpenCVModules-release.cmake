#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "opencv_cudev" for configuration "Release"
set_property(TARGET opencv_cudev APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(opencv_cudev PROPERTIES
  IMPORTED_IMPLIB_RELEASE "${_IMPORT_PREFIX}/x86/vc11/lib/opencv_cudev300.lib"
  IMPORTED_LINK_INTERFACE_LIBRARIES_RELEASE ""
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/x86/vc11/bin/opencv_cudev300.dll"
  )

list(APPEND _IMPORT_CHECK_TARGETS opencv_cudev )
list(APPEND _IMPORT_CHECK_FILES_FOR_opencv_cudev "${_IMPORT_PREFIX}/x86/vc11/lib/opencv_cudev300.lib" "${_IMPORT_PREFIX}/x86/vc11/bin/opencv_cudev300.dll" )

# Import target "opencv_core" for configuration "Release"
set_property(TARGET opencv_core APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(opencv_core PROPERTIES
  IMPORTED_IMPLIB_RELEASE "${_IMPORT_PREFIX}/x86/vc11/lib/opencv_core300.lib"
  IMPORTED_LINK_INTERFACE_LIBRARIES_RELEASE "opencv_cudev"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/x86/vc11/bin/opencv_core300.dll"
  )

list(APPEND _IMPORT_CHECK_TARGETS opencv_core )
list(APPEND _IMPORT_CHECK_FILES_FOR_opencv_core "${_IMPORT_PREFIX}/x86/vc11/lib/opencv_core300.lib" "${_IMPORT_PREFIX}/x86/vc11/bin/opencv_core300.dll" )

# Import target "opencv_flann" for configuration "Release"
set_property(TARGET opencv_flann APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(opencv_flann PROPERTIES
  IMPORTED_IMPLIB_RELEASE "${_IMPORT_PREFIX}/x86/vc11/lib/opencv_flann300.lib"
  IMPORTED_LINK_INTERFACE_LIBRARIES_RELEASE "opencv_cudev;opencv_core"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/x86/vc11/bin/opencv_flann300.dll"
  )

list(APPEND _IMPORT_CHECK_TARGETS opencv_flann )
list(APPEND _IMPORT_CHECK_FILES_FOR_opencv_flann "${_IMPORT_PREFIX}/x86/vc11/lib/opencv_flann300.lib" "${_IMPORT_PREFIX}/x86/vc11/bin/opencv_flann300.dll" )

# Import target "opencv_imgproc" for configuration "Release"
set_property(TARGET opencv_imgproc APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(opencv_imgproc PROPERTIES
  IMPORTED_IMPLIB_RELEASE "${_IMPORT_PREFIX}/x86/vc11/lib/opencv_imgproc300.lib"
  IMPORTED_LINK_INTERFACE_LIBRARIES_RELEASE "opencv_cudev;opencv_core"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/x86/vc11/bin/opencv_imgproc300.dll"
  )

list(APPEND _IMPORT_CHECK_TARGETS opencv_imgproc )
list(APPEND _IMPORT_CHECK_FILES_FOR_opencv_imgproc "${_IMPORT_PREFIX}/x86/vc11/lib/opencv_imgproc300.lib" "${_IMPORT_PREFIX}/x86/vc11/bin/opencv_imgproc300.dll" )

# Import target "opencv_imgcodecs" for configuration "Release"
set_property(TARGET opencv_imgcodecs APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(opencv_imgcodecs PROPERTIES
  IMPORTED_IMPLIB_RELEASE "${_IMPORT_PREFIX}/x86/vc11/lib/opencv_imgcodecs300.lib"
  IMPORTED_LINK_DEPENDENT_LIBRARIES_RELEASE "opencv_cudev;opencv_core;opencv_imgproc"
  IMPORTED_LINK_INTERFACE_LIBRARIES_RELEASE ""
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/x86/vc11/bin/opencv_imgcodecs300.dll"
  )

list(APPEND _IMPORT_CHECK_TARGETS opencv_imgcodecs )
list(APPEND _IMPORT_CHECK_FILES_FOR_opencv_imgcodecs "${_IMPORT_PREFIX}/x86/vc11/lib/opencv_imgcodecs300.lib" "${_IMPORT_PREFIX}/x86/vc11/bin/opencv_imgcodecs300.dll" )

# Import target "opencv_videoio" for configuration "Release"
set_property(TARGET opencv_videoio APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(opencv_videoio PROPERTIES
  IMPORTED_IMPLIB_RELEASE "${_IMPORT_PREFIX}/x86/vc11/lib/opencv_videoio300.lib"
  IMPORTED_LINK_DEPENDENT_LIBRARIES_RELEASE "opencv_cudev;opencv_core;opencv_imgproc;opencv_imgcodecs"
  IMPORTED_LINK_INTERFACE_LIBRARIES_RELEASE ""
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/x86/vc11/bin/opencv_videoio300.dll"
  )

list(APPEND _IMPORT_CHECK_TARGETS opencv_videoio )
list(APPEND _IMPORT_CHECK_FILES_FOR_opencv_videoio "${_IMPORT_PREFIX}/x86/vc11/lib/opencv_videoio300.lib" "${_IMPORT_PREFIX}/x86/vc11/bin/opencv_videoio300.dll" )

# Import target "opencv_highgui" for configuration "Release"
set_property(TARGET opencv_highgui APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(opencv_highgui PROPERTIES
  IMPORTED_IMPLIB_RELEASE "${_IMPORT_PREFIX}/x86/vc11/lib/opencv_highgui300.lib"
  IMPORTED_LINK_DEPENDENT_LIBRARIES_RELEASE "opencv_cudev;opencv_core;opencv_imgproc;opencv_imgcodecs;opencv_videoio"
  IMPORTED_LINK_INTERFACE_LIBRARIES_RELEASE ""
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/x86/vc11/bin/opencv_highgui300.dll"
  )

list(APPEND _IMPORT_CHECK_TARGETS opencv_highgui )
list(APPEND _IMPORT_CHECK_FILES_FOR_opencv_highgui "${_IMPORT_PREFIX}/x86/vc11/lib/opencv_highgui300.lib" "${_IMPORT_PREFIX}/x86/vc11/bin/opencv_highgui300.dll" )

# Import target "opencv_ml" for configuration "Release"
set_property(TARGET opencv_ml APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(opencv_ml PROPERTIES
  IMPORTED_IMPLIB_RELEASE "${_IMPORT_PREFIX}/x86/vc11/lib/opencv_ml300.lib"
  IMPORTED_LINK_INTERFACE_LIBRARIES_RELEASE "opencv_cudev;opencv_core"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/x86/vc11/bin/opencv_ml300.dll"
  )

list(APPEND _IMPORT_CHECK_TARGETS opencv_ml )
list(APPEND _IMPORT_CHECK_FILES_FOR_opencv_ml "${_IMPORT_PREFIX}/x86/vc11/lib/opencv_ml300.lib" "${_IMPORT_PREFIX}/x86/vc11/bin/opencv_ml300.dll" )

# Import target "opencv_features2d" for configuration "Release"
set_property(TARGET opencv_features2d APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(opencv_features2d PROPERTIES
  IMPORTED_IMPLIB_RELEASE "${_IMPORT_PREFIX}/x86/vc11/lib/opencv_features2d300.lib"
  IMPORTED_LINK_INTERFACE_LIBRARIES_RELEASE "opencv_cudev;opencv_core;opencv_flann;opencv_imgproc;opencv_imgcodecs;opencv_videoio;opencv_highgui;opencv_ml"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/x86/vc11/bin/opencv_features2d300.dll"
  )

list(APPEND _IMPORT_CHECK_TARGETS opencv_features2d )
list(APPEND _IMPORT_CHECK_FILES_FOR_opencv_features2d "${_IMPORT_PREFIX}/x86/vc11/lib/opencv_features2d300.lib" "${_IMPORT_PREFIX}/x86/vc11/bin/opencv_features2d300.dll" )

# Import target "opencv_calib3d" for configuration "Release"
set_property(TARGET opencv_calib3d APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(opencv_calib3d PROPERTIES
  IMPORTED_IMPLIB_RELEASE "${_IMPORT_PREFIX}/x86/vc11/lib/opencv_calib3d300.lib"
  IMPORTED_LINK_INTERFACE_LIBRARIES_RELEASE "opencv_cudev;opencv_core;opencv_flann;opencv_imgproc;opencv_imgcodecs;opencv_videoio;opencv_highgui;opencv_ml;opencv_features2d"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/x86/vc11/bin/opencv_calib3d300.dll"
  )

list(APPEND _IMPORT_CHECK_TARGETS opencv_calib3d )
list(APPEND _IMPORT_CHECK_FILES_FOR_opencv_calib3d "${_IMPORT_PREFIX}/x86/vc11/lib/opencv_calib3d300.lib" "${_IMPORT_PREFIX}/x86/vc11/bin/opencv_calib3d300.dll" )

# Import target "opencv_cudaarithm" for configuration "Release"
set_property(TARGET opencv_cudaarithm APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(opencv_cudaarithm PROPERTIES
  IMPORTED_IMPLIB_RELEASE "${_IMPORT_PREFIX}/x86/vc11/lib/opencv_cudaarithm300.lib"
  IMPORTED_LINK_INTERFACE_LIBRARIES_RELEASE "opencv_cudev;opencv_core"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/x86/vc11/bin/opencv_cudaarithm300.dll"
  )

list(APPEND _IMPORT_CHECK_TARGETS opencv_cudaarithm )
list(APPEND _IMPORT_CHECK_FILES_FOR_opencv_cudaarithm "${_IMPORT_PREFIX}/x86/vc11/lib/opencv_cudaarithm300.lib" "${_IMPORT_PREFIX}/x86/vc11/bin/opencv_cudaarithm300.dll" )

# Import target "opencv_objdetect" for configuration "Release"
set_property(TARGET opencv_objdetect APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(opencv_objdetect PROPERTIES
  IMPORTED_IMPLIB_RELEASE "${_IMPORT_PREFIX}/x86/vc11/lib/opencv_objdetect300.lib"
  IMPORTED_LINK_INTERFACE_LIBRARIES_RELEASE "opencv_cudev;opencv_core;opencv_imgproc;opencv_imgcodecs;opencv_videoio;opencv_highgui;opencv_ml"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/x86/vc11/bin/opencv_objdetect300.dll"
  )

list(APPEND _IMPORT_CHECK_TARGETS opencv_objdetect )
list(APPEND _IMPORT_CHECK_FILES_FOR_opencv_objdetect "${_IMPORT_PREFIX}/x86/vc11/lib/opencv_objdetect300.lib" "${_IMPORT_PREFIX}/x86/vc11/bin/opencv_objdetect300.dll" )

# Import target "opencv_cudalegacy" for configuration "Release"
set_property(TARGET opencv_cudalegacy APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(opencv_cudalegacy PROPERTIES
  IMPORTED_IMPLIB_RELEASE "${_IMPORT_PREFIX}/x86/vc11/lib/opencv_cudalegacy300.lib"
  IMPORTED_LINK_INTERFACE_LIBRARIES_RELEASE "opencv_cudev;opencv_core;opencv_imgproc;opencv_imgcodecs;opencv_videoio;opencv_highgui;opencv_ml;opencv_objdetect"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/x86/vc11/bin/opencv_cudalegacy300.dll"
  )

list(APPEND _IMPORT_CHECK_TARGETS opencv_cudalegacy )
list(APPEND _IMPORT_CHECK_FILES_FOR_opencv_cudalegacy "${_IMPORT_PREFIX}/x86/vc11/lib/opencv_cudalegacy300.lib" "${_IMPORT_PREFIX}/x86/vc11/bin/opencv_cudalegacy300.dll" )

# Import target "opencv_cudawarping" for configuration "Release"
set_property(TARGET opencv_cudawarping APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(opencv_cudawarping PROPERTIES
  IMPORTED_IMPLIB_RELEASE "${_IMPORT_PREFIX}/x86/vc11/lib/opencv_cudawarping300.lib"
  IMPORTED_LINK_INTERFACE_LIBRARIES_RELEASE "opencv_cudev;opencv_core;opencv_imgproc;opencv_imgcodecs;opencv_videoio;opencv_highgui;opencv_ml;opencv_objdetect;opencv_cudalegacy"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/x86/vc11/bin/opencv_cudawarping300.dll"
  )

list(APPEND _IMPORT_CHECK_TARGETS opencv_cudawarping )
list(APPEND _IMPORT_CHECK_FILES_FOR_opencv_cudawarping "${_IMPORT_PREFIX}/x86/vc11/lib/opencv_cudawarping300.lib" "${_IMPORT_PREFIX}/x86/vc11/bin/opencv_cudawarping300.dll" )

# Import target "opencv_cuda" for configuration "Release"
set_property(TARGET opencv_cuda APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(opencv_cuda PROPERTIES
  IMPORTED_IMPLIB_RELEASE "${_IMPORT_PREFIX}/x86/vc11/lib/opencv_cuda300.lib"
  IMPORTED_LINK_INTERFACE_LIBRARIES_RELEASE "opencv_cudev;opencv_core;opencv_flann;opencv_imgproc;opencv_imgcodecs;opencv_videoio;opencv_highgui;opencv_ml;opencv_features2d;opencv_calib3d;opencv_cudaarithm;opencv_objdetect;opencv_cudalegacy;opencv_cudawarping"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/x86/vc11/bin/opencv_cuda300.dll"
  )

list(APPEND _IMPORT_CHECK_TARGETS opencv_cuda )
list(APPEND _IMPORT_CHECK_FILES_FOR_opencv_cuda "${_IMPORT_PREFIX}/x86/vc11/lib/opencv_cuda300.lib" "${_IMPORT_PREFIX}/x86/vc11/bin/opencv_cuda300.dll" )

# Import target "opencv_cudafilters" for configuration "Release"
set_property(TARGET opencv_cudafilters APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(opencv_cudafilters PROPERTIES
  IMPORTED_IMPLIB_RELEASE "${_IMPORT_PREFIX}/x86/vc11/lib/opencv_cudafilters300.lib"
  IMPORTED_LINK_INTERFACE_LIBRARIES_RELEASE "opencv_cudev;opencv_core;opencv_cudaarithm;opencv_imgproc"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/x86/vc11/bin/opencv_cudafilters300.dll"
  )

list(APPEND _IMPORT_CHECK_TARGETS opencv_cudafilters )
list(APPEND _IMPORT_CHECK_FILES_FOR_opencv_cudafilters "${_IMPORT_PREFIX}/x86/vc11/lib/opencv_cudafilters300.lib" "${_IMPORT_PREFIX}/x86/vc11/bin/opencv_cudafilters300.dll" )

# Import target "opencv_cudaimgproc" for configuration "Release"
set_property(TARGET opencv_cudaimgproc APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(opencv_cudaimgproc PROPERTIES
  IMPORTED_IMPLIB_RELEASE "${_IMPORT_PREFIX}/x86/vc11/lib/opencv_cudaimgproc300.lib"
  IMPORTED_LINK_INTERFACE_LIBRARIES_RELEASE "opencv_cudev;opencv_core;opencv_cudaarithm;opencv_imgproc;opencv_cudafilters"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/x86/vc11/bin/opencv_cudaimgproc300.dll"
  )

list(APPEND _IMPORT_CHECK_TARGETS opencv_cudaimgproc )
list(APPEND _IMPORT_CHECK_FILES_FOR_opencv_cudaimgproc "${_IMPORT_PREFIX}/x86/vc11/lib/opencv_cudaimgproc300.lib" "${_IMPORT_PREFIX}/x86/vc11/bin/opencv_cudaimgproc300.dll" )

# Import target "opencv_video" for configuration "Release"
set_property(TARGET opencv_video APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(opencv_video PROPERTIES
  IMPORTED_IMPLIB_RELEASE "${_IMPORT_PREFIX}/x86/vc11/lib/opencv_video300.lib"
  IMPORTED_LINK_INTERFACE_LIBRARIES_RELEASE "opencv_cudev;opencv_core;opencv_imgproc"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/x86/vc11/bin/opencv_video300.dll"
  )

list(APPEND _IMPORT_CHECK_TARGETS opencv_video )
list(APPEND _IMPORT_CHECK_FILES_FOR_opencv_video "${_IMPORT_PREFIX}/x86/vc11/lib/opencv_video300.lib" "${_IMPORT_PREFIX}/x86/vc11/bin/opencv_video300.dll" )

# Import target "opencv_cudabgsegm" for configuration "Release"
set_property(TARGET opencv_cudabgsegm APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(opencv_cudabgsegm PROPERTIES
  IMPORTED_IMPLIB_RELEASE "${_IMPORT_PREFIX}/x86/vc11/lib/opencv_cudabgsegm300.lib"
  IMPORTED_LINK_INTERFACE_LIBRARIES_RELEASE "opencv_cudev;opencv_core;opencv_cudaarithm;opencv_imgproc;opencv_cudafilters;opencv_cudaimgproc;opencv_video"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/x86/vc11/bin/opencv_cudabgsegm300.dll"
  )

list(APPEND _IMPORT_CHECK_TARGETS opencv_cudabgsegm )
list(APPEND _IMPORT_CHECK_FILES_FOR_opencv_cudabgsegm "${_IMPORT_PREFIX}/x86/vc11/lib/opencv_cudabgsegm300.lib" "${_IMPORT_PREFIX}/x86/vc11/bin/opencv_cudabgsegm300.dll" )

# Import target "opencv_cudacodec" for configuration "Release"
set_property(TARGET opencv_cudacodec APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(opencv_cudacodec PROPERTIES
  IMPORTED_IMPLIB_RELEASE "${_IMPORT_PREFIX}/x86/vc11/lib/opencv_cudacodec300.lib"
  IMPORTED_LINK_INTERFACE_LIBRARIES_RELEASE "opencv_cudev;opencv_core;opencv_imgproc;opencv_imgcodecs;opencv_videoio"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/x86/vc11/bin/opencv_cudacodec300.dll"
  )

list(APPEND _IMPORT_CHECK_TARGETS opencv_cudacodec )
list(APPEND _IMPORT_CHECK_FILES_FOR_opencv_cudacodec "${_IMPORT_PREFIX}/x86/vc11/lib/opencv_cudacodec300.lib" "${_IMPORT_PREFIX}/x86/vc11/bin/opencv_cudacodec300.dll" )

# Import target "opencv_cudafeatures2d" for configuration "Release"
set_property(TARGET opencv_cudafeatures2d APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(opencv_cudafeatures2d PROPERTIES
  IMPORTED_IMPLIB_RELEASE "${_IMPORT_PREFIX}/x86/vc11/lib/opencv_cudafeatures2d300.lib"
  IMPORTED_LINK_INTERFACE_LIBRARIES_RELEASE "opencv_cudev;opencv_core;opencv_cudaarithm;opencv_imgproc;opencv_cudafilters;opencv_imgcodecs;opencv_videoio;opencv_highgui;opencv_ml;opencv_objdetect;opencv_cudalegacy;opencv_cudawarping;opencv_flann;opencv_features2d"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/x86/vc11/bin/opencv_cudafeatures2d300.dll"
  )

list(APPEND _IMPORT_CHECK_TARGETS opencv_cudafeatures2d )
list(APPEND _IMPORT_CHECK_FILES_FOR_opencv_cudafeatures2d "${_IMPORT_PREFIX}/x86/vc11/lib/opencv_cudafeatures2d300.lib" "${_IMPORT_PREFIX}/x86/vc11/bin/opencv_cudafeatures2d300.dll" )

# Import target "opencv_cudaoptflow" for configuration "Release"
set_property(TARGET opencv_cudaoptflow APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(opencv_cudaoptflow PROPERTIES
  IMPORTED_IMPLIB_RELEASE "${_IMPORT_PREFIX}/x86/vc11/lib/opencv_cudaoptflow300.lib"
  IMPORTED_LINK_INTERFACE_LIBRARIES_RELEASE "opencv_cudev;opencv_core;opencv_cudaarithm;opencv_imgproc;opencv_cudafilters;opencv_cudaimgproc;opencv_imgcodecs;opencv_videoio;opencv_highgui;opencv_ml;opencv_objdetect;opencv_cudalegacy;opencv_cudawarping;opencv_video"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/x86/vc11/bin/opencv_cudaoptflow300.dll"
  )

list(APPEND _IMPORT_CHECK_TARGETS opencv_cudaoptflow )
list(APPEND _IMPORT_CHECK_FILES_FOR_opencv_cudaoptflow "${_IMPORT_PREFIX}/x86/vc11/lib/opencv_cudaoptflow300.lib" "${_IMPORT_PREFIX}/x86/vc11/bin/opencv_cudaoptflow300.dll" )

# Import target "opencv_cudastereo" for configuration "Release"
set_property(TARGET opencv_cudastereo APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(opencv_cudastereo PROPERTIES
  IMPORTED_IMPLIB_RELEASE "${_IMPORT_PREFIX}/x86/vc11/lib/opencv_cudastereo300.lib"
  IMPORTED_LINK_INTERFACE_LIBRARIES_RELEASE "opencv_cudev;opencv_core;opencv_flann;opencv_imgproc;opencv_imgcodecs;opencv_videoio;opencv_highgui;opencv_ml;opencv_features2d;opencv_calib3d"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/x86/vc11/bin/opencv_cudastereo300.dll"
  )

list(APPEND _IMPORT_CHECK_TARGETS opencv_cudastereo )
list(APPEND _IMPORT_CHECK_FILES_FOR_opencv_cudastereo "${_IMPORT_PREFIX}/x86/vc11/lib/opencv_cudastereo300.lib" "${_IMPORT_PREFIX}/x86/vc11/bin/opencv_cudastereo300.dll" )

# Import target "opencv_photo" for configuration "Release"
set_property(TARGET opencv_photo APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(opencv_photo PROPERTIES
  IMPORTED_IMPLIB_RELEASE "${_IMPORT_PREFIX}/x86/vc11/lib/opencv_photo300.lib"
  IMPORTED_LINK_INTERFACE_LIBRARIES_RELEASE "opencv_cudev;opencv_core;opencv_cudaarithm;opencv_imgproc;opencv_cudafilters;opencv_cudaimgproc"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/x86/vc11/bin/opencv_photo300.dll"
  )

list(APPEND _IMPORT_CHECK_TARGETS opencv_photo )
list(APPEND _IMPORT_CHECK_FILES_FOR_opencv_photo "${_IMPORT_PREFIX}/x86/vc11/lib/opencv_photo300.lib" "${_IMPORT_PREFIX}/x86/vc11/bin/opencv_photo300.dll" )

# Import target "opencv_shape" for configuration "Release"
set_property(TARGET opencv_shape APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(opencv_shape PROPERTIES
  IMPORTED_IMPLIB_RELEASE "${_IMPORT_PREFIX}/x86/vc11/lib/opencv_shape300.lib"
  IMPORTED_LINK_INTERFACE_LIBRARIES_RELEASE "opencv_cudev;opencv_core;opencv_imgproc;opencv_video"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/x86/vc11/bin/opencv_shape300.dll"
  )

list(APPEND _IMPORT_CHECK_TARGETS opencv_shape )
list(APPEND _IMPORT_CHECK_FILES_FOR_opencv_shape "${_IMPORT_PREFIX}/x86/vc11/lib/opencv_shape300.lib" "${_IMPORT_PREFIX}/x86/vc11/bin/opencv_shape300.dll" )

# Import target "opencv_stitching" for configuration "Release"
set_property(TARGET opencv_stitching APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(opencv_stitching PROPERTIES
  IMPORTED_IMPLIB_RELEASE "${_IMPORT_PREFIX}/x86/vc11/lib/opencv_stitching300.lib"
  IMPORTED_LINK_INTERFACE_LIBRARIES_RELEASE "opencv_cudev;opencv_core;opencv_flann;opencv_imgproc;opencv_imgcodecs;opencv_videoio;opencv_highgui;opencv_ml;opencv_features2d;opencv_calib3d;opencv_cudaarithm;opencv_objdetect;opencv_cudalegacy;opencv_cudawarping;opencv_cuda;opencv_cudafilters;opencv_cudafeatures2d"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/x86/vc11/bin/opencv_stitching300.dll"
  )

list(APPEND _IMPORT_CHECK_TARGETS opencv_stitching )
list(APPEND _IMPORT_CHECK_FILES_FOR_opencv_stitching "${_IMPORT_PREFIX}/x86/vc11/lib/opencv_stitching300.lib" "${_IMPORT_PREFIX}/x86/vc11/bin/opencv_stitching300.dll" )

# Import target "opencv_superres" for configuration "Release"
set_property(TARGET opencv_superres APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(opencv_superres PROPERTIES
  IMPORTED_IMPLIB_RELEASE "${_IMPORT_PREFIX}/x86/vc11/lib/opencv_superres300.lib"
  IMPORTED_LINK_INTERFACE_LIBRARIES_RELEASE "opencv_cudev;opencv_core;opencv_cudaarithm;opencv_imgproc;opencv_imgcodecs;opencv_videoio;opencv_cudacodec;opencv_cudafilters;opencv_cudaimgproc;opencv_highgui;opencv_ml;opencv_objdetect;opencv_cudalegacy;opencv_cudawarping;opencv_video;opencv_cudaoptflow"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/x86/vc11/bin/opencv_superres300.dll"
  )

list(APPEND _IMPORT_CHECK_TARGETS opencv_superres )
list(APPEND _IMPORT_CHECK_FILES_FOR_opencv_superres "${_IMPORT_PREFIX}/x86/vc11/lib/opencv_superres300.lib" "${_IMPORT_PREFIX}/x86/vc11/bin/opencv_superres300.dll" )

# Import target "opencv_ts" for configuration "Release"
set_property(TARGET opencv_ts APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(opencv_ts PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_RELEASE "CXX"
  IMPORTED_LINK_INTERFACE_LIBRARIES_RELEASE "opencv_cudev;opencv_core;opencv_imgproc;opencv_imgcodecs;opencv_videoio;opencv_highgui;ippicv;cudart;nppc;nppi;npps;cufft;cudart;nppc;nppi;npps"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/x86/vc11/lib/opencv_ts300.lib"
  )

list(APPEND _IMPORT_CHECK_TARGETS opencv_ts )
list(APPEND _IMPORT_CHECK_FILES_FOR_opencv_ts "${_IMPORT_PREFIX}/x86/vc11/lib/opencv_ts300.lib" )

# Import target "opencv_videostab" for configuration "Release"
set_property(TARGET opencv_videostab APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(opencv_videostab PROPERTIES
  IMPORTED_IMPLIB_RELEASE "${_IMPORT_PREFIX}/x86/vc11/lib/opencv_videostab300.lib"
  IMPORTED_LINK_INTERFACE_LIBRARIES_RELEASE "opencv_cudev;opencv_core;opencv_flann;opencv_imgproc;opencv_imgcodecs;opencv_videoio;opencv_highgui;opencv_ml;opencv_features2d;opencv_calib3d;opencv_cudaarithm;opencv_objdetect;opencv_cudalegacy;opencv_cudawarping;opencv_cuda;opencv_cudafilters;opencv_cudaimgproc;opencv_video;opencv_cudaoptflow;opencv_photo"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/x86/vc11/bin/opencv_videostab300.dll"
  )

list(APPEND _IMPORT_CHECK_TARGETS opencv_videostab )
list(APPEND _IMPORT_CHECK_FILES_FOR_opencv_videostab "${_IMPORT_PREFIX}/x86/vc11/lib/opencv_videostab300.lib" "${_IMPORT_PREFIX}/x86/vc11/bin/opencv_videostab300.dll" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
