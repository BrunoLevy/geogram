
add_subdirectory(vorpastat)
add_subdirectory(vorpacomp)
add_subdirectory(geodump)
add_subdirectory(geocsg)

if(GEOGRAM_WITH_GRAPHICS)
   add_subdirectory(vorpaview)
   add_subdirectory(geobox)
   add_subdirectory(geoshade)
   add_subdirectory(geomol)
   if(GEOGRAM_WITH_LUA)
      add_subdirectory(geocod)
   endif()
endif()

add_subdirectory(vorpalite)

# if(GEOGRAM_WITH_VORPALINE)
#  add_subdirectory(vorpaline)
# endif()

if(GEOGRAM_WITH_FPG)
   add_subdirectory(fpg)
   add_subdirectory(pck)
endif()
