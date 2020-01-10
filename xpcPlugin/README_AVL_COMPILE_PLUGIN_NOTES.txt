Compile XPConnect plugin with new methods from XPLM210 the Preprocessor Definition needs to be set in the VS solution:


Project -> xpcPlugin Properties...

Configuration: Release
Platform: x64

Configuration Properties -> C/C++ -> Preprozessor -> Preprocessor Definitions: IBM=1;XPLM210;%(PreprocessorDefinitions)

Afterwards the new methods like XPLMCreateFlightLoop are available in XPLMProcessing.h



COMPILE PLUGIN:

The plugin is created in the folder "\xpcPlugin", copy folder "XPlaneConnect" to "\X-Plane 11\Resources\plugins"



