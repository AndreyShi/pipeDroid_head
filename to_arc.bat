for %%a in (".") do set CURRENT_DIR_NAME=%%~na
IF EXIST %cd%\%CURRENT_DIR_NAME%.zip (
	del %cd%\%CURRENT_DIR_NAME%.zip
)
start "" "C:\Program Files\7-Zip\7z.exe" a -tzip -mx5 -r0 ^
-xr!.vs -xr!.git -xr!.hg -xr!debug -xr!release -xr!*.zip %cd%\%CURRENT_DIR_NAME%.zip %cd%\*
