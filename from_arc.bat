@echo off
for %%a in (".") do set CURRENT_DIR_NAME=%%~na
IF EXIST %cd%\%CURRENT_DIR_NAME%.zip (
	start "" /wait "C:\Program Files\7-Zip\7z.exe" x -y %cd%\%CURRENT_DIR_NAME%.zip -o"%cd%"
	del %cd%\%CURRENT_DIR_NAME%.zip
) ELSE (
	echo %cd%\%CURRENT_DIR_NAME%.zip not found
	pause
)
