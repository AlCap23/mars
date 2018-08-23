
set(FMILibrary_ROOT, /media/jmartensen/Data/linux/fmi_library/FMILibrary/install)


find_path(FMILibrary_INCLUDE_DIR fmilib.h
          HINTS
          $ENV{FMILibrary_ROOT}/include
          /media/jmartensen/Data/linux/fmi_library/FMILibrary/install/include
          )

find_library(FMILibrary_LIBRARY
             NAMES fmilib_shared libfmilib_shared
             HINTS
             $ENV{FMILibrary_ROOT}/lib
             /media/jmartensen/Data/linux/fmi_library/FMILibrary/install/lib
             )

mark_as_advanced(FMILibrary_INCLUDE_DIR
                 FMILibrary_LIBRARY)

if(FMILibrary)
  message(STATUS, "\n Found FMI Library \n")
  message(STATUS, ${FMILibrary_INCLUDE_DIR})
else()
  message(STATUS, "\n No FMI Library found \n")
endif()

if(FMILibrary_INCLUDE_DIR)
  message(STATUS, "\n No include directory found for FMI Library \n")
endif()

if(FMILibrary_LIBRARY AND FMILibrary_INCLUDE_DIR AND NOT TARGET FMILibrary::FMILibrary)
    message(STATUS, "Found something")
    add_library(FMILibrary::FMILibrary UNKNOWN IMPORTED)
    set_target_properties(FMILibrary::FMILibrary PROPERTIES
                          INTERFACE_INCLUDE_DIRECTORIES "${FMILibrary_INCLUDE_DIR}"
                          IMPORTED_LOCATION "${FMILibrary_LIBRARY}")

    set(FMILibrary_LIBRARIES FMILibrary::FMILibrary)
    set(FMILibrary_INCLUDE_DIRS "${FMILibrary_INCLUDE_DIR}")

    include(FindPackageHandleStandardArgs)
    find_package_handle_standard_args(FMILibrary
                                      FOUND_VAR FMILibrary_FOUND
                                      REQUIRED_VARS FMILibrary_LIBRARIES FMILibrary_INCLUDE_DIRS)

    # Set package properties if FeatureSummary was included
    if(COMMAND set_package_properties)
        set_package_properties(FMILibrary PROPERTIES DESCRIPTION "FMILibrary"
                                                     URL "http://www.jmodelica.org/FMILibrary")
    endif()

endif()
