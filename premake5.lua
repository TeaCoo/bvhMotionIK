workspace "IK_processer"
	architecture "x64"
	configurations
	{
		"Debug",
		"Release",
	}
	startproject "IK_processer"

outputdir = "%{cfg.buildcfg}-%{cfg.system}-%{cfg.architecture}"

project "IK_processer"
	location "IK_processer"
	kind "ConsoleApp"
	language "C++"
	targetdir ("bin/" .. outputdir .. "/%{prj.name}")
	objdir ("bin-int/" .. outputdir .. "/%{prj.name}")
	files
	{
		"%{prj.name}/include/**.h",
		"%{prj.name}/src/**.cpp"
	}

	includedirs
	{
		"IK_processer/include",
		"IK_processer/extern/eigen-3.4.0",
	}

	filter "system:windows"
		cppdialect "C++17"
		staticruntime "On"
		systemversion "latest"