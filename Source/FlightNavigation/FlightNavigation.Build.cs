// Copyright Epic Games, Inc. All Rights Reserved.

using UnrealBuildTool;

public class FlightNavigation : ModuleRules
{
	public FlightNavigation(ReadOnlyTargetRules Target) : base(Target)
	{
		PCHUsage = ModuleRules.PCHUsageMode.UseExplicitOrSharedPCHs;
		
		PublicIncludePaths.AddRange(
			new string[] {
				// ... add public include paths required here ...
			}
			);
				
		
		PrivateIncludePaths.AddRange(
			new string[] {
				// ... add other private include paths required here ...
			}
			);
			
		
		PublicDependencyModuleNames.AddRange(
			new string[]
			{
				"Core",
				"CoreUObject",
				"Engine",
				"NavigationSystem",
				// ... add other public dependencies that you statically link with here ...
			}
			);
		
		if (Target.bBuildEditor)
		{
			PublicDependencyModuleNames.AddRange(new[] {"UnrealEd"});
		}
		
		PrivateDependencyModuleNames.AddRange(
			new string[]
			{
				"RHI",
				"RenderCore"
				// ... add private dependencies that you statically link with here ...	
			}
			);
		
		
		DynamicallyLoadedModuleNames.AddRange(
			new string[]
			{
				// ... add any modules that your module loads dynamically here ...
			}
			);
	}
}
