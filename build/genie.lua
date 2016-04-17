
function SetTarget( _configuration, _platform, _basepath )
	--print(_configuration .. _platform)
	local platformname = _platform
	local archname = _platform
	if _platform == "x32" then
		platformname = "Win32"
		archname = "x86"
	end
	local strtarget = string.format( "%s/bin/%s_%s/", _basepath, _configuration, platformname ) 
	local strobj = string.format( "%s/intermediate/%s_%s", _basepath, _configuration, platformname ) 
	configuration {_configuration, _platform}
		targetdir( strtarget )
		objdir( strobj )
end

function SetLibs( _configuration, _platform, _basepath )
	local platformname = _platform
	local archname = _platform
	if _platform == "x32" then
		platformname = "Win32"
		archname = "x86"
	end

	local strGlew
	if _platform == "native" then
		strGlew = string.format( "%s/3rdparty/glew-1.13.0/lib", _basepath )
	else
		strGlew = string.format( "%s/3rdparty/glew-1.13.0/lib/release/%s", _basepath, platformname )
	end

	local strSdl2 = string.format( "%s/3rdparty/SDL2-2.0.3/lib/%s/%s", _basepath, platformname, _configuration )
	local strSdl2Options = string.format( "-F %s/3rdparty/SDL2-2.0.3/lib/%s/%s", _basepath, platformname, _configuration )
	
	-- add framework search path for SDL2
	if _platform == "native" then
		configuration {_configuration, _platform}
	    	linkoptions  { strSdl2Options }
	end

	local strImgui = string.format( "%s/3rdparty/imgui/bin/%s_%s", _basepath, platformname, _configuration )
	local strJsmn = string.format( "%s/3rdparty/jsmn/bin/%s_%s", _basepath, platformname, _configuration )

	configuration {_configuration, _platform}
		libdirs { strGlew, strSdl2, strImgui, strJsmn }
end


--------------------------------------------------------
solution "sae"
	configurations { "Debug", "Release" }

	configuration "macosx"
		platforms { "native" }
	configuration "windows"
		platforms { "x32", "x64" }
		
	--filter { "platforms:x64" }
	--	system "Windows"
	--	architecture "x64"
		
	--filter { "platforms:x32" }
	--	system "Windows"
	--	architecture "x32"

	---------------------------------------------------------
	project "bigball" 
		kind "StaticLib"
		language "C++"
		files { "../../engine/src/**.h", "../../engine/src/**.cpp" }

		includedirs { "../../3rdparty/SDL2-2.0.3/include", "../../3rdparty/glew-1.13.0/include", "../../3rdparty/zlib-1.2.8", "../../3rdparty/jsmn", "../../3rdparty/imgui", "../../3rdparty" }

		defines { "_CRT_SECURE_NO_WARNINGS" }
		
		local targetpath = "../../engine"
		configuration "windows"
			SetTarget( "Debug", "x32", targetpath )
			SetTarget( "Debug", "x64", targetpath )
			SetTarget( "Release", "x32", targetpath )
			SetTarget( "Release", "x64", targetpath )
			
		configuration "macosx"
			SetTarget( "Debug", "native", targetpath )
			SetTarget( "Release", "native", targetpath )
			
		configuration "Debug"
			defines { "_DEBUG" }
			flags { "Symbols" }
 
		configuration "Release"
			defines { "NDEBUG" }
			flags { "Optimize", "Symbols" }
			--optimize "On"

		configuration "macosx"
            buildoptions { "-std=c++11" } --, "-stdlib=libc++" }

	---------------------------------------------------------
	project "sae"
		kind "ConsoleApp"
		language "C++"
		files { "../src/**.h", "../src/**.cpp" }

		--removefiles { "3rdparty/**", "mars**" }
		targetname "sae"
	
		defines { "_CRT_SECURE_NO_WARNINGS", "_WINDOWS", "_USRDLL" }
		flags { "NoPCH", "NoNativeWChar", "NoEditAndContinue" }

		includedirs { "../../engine/src/", "../../3rdparty", "../../3rdparty/SDL2-2.0.3/include", "../../3rdparty/glew-1.13.0/include", "../../3rdparty/zlib-1.2.8", "../../3rdparty/jsmn", "../../3rdparty/imgui", "../../3rdparty/eigen3", "../data/shader" }

		configuration "windows"	
			links { "bigball", "glew32", "sdl2", "sdl2main", "opengl32", "imgui", "jsmn" }

		configuration "macosx"
			links { "bigball", "glew", "SDL2.framework", "OpenGL.framework", "imgui", "jsmn" }

		local targetpath = ".."
		local libpath = "../.."
		configuration "windows"
			SetTarget( "Debug", "x32", targetpath )
			SetTarget( "Debug", "x64", targetpath )
			SetTarget( "Release", "x32", targetpath )
			SetTarget( "Release", "x64", targetpath )
			SetLibs( "Debug", "x32", libpath )
			SetLibs( "Debug", "x64", libpath )
			SetLibs( "Release", "x32", libpath )
			SetLibs( "Release", "x64", libpath )
			
		configuration "macosx"
			SetTarget( "Debug", "native", targetpath )
			SetTarget( "Release", "native", targetpath )
			SetLibs( "Debug", "native", libpath )
			SetLibs( "Release", "native", libpath )

		configuration "Debug"
			defines { "_DEBUG" }
			flags { "Symbols" }
 
		configuration "Release"
			defines { "NDEBUG" }
			flags { "Optimize", "Symbols"}
			--optimize "On"

		configuration "macosx"
            linkoptions  { "-std=c++11" } 
            buildoptions { "-std=c++11" }
			

