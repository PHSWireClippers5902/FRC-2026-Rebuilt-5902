@REM blame Daniel Sabalakov for this abismal creation...
@REM AAAAAAHHHHHHHH
@echo off 
git add *
git commit -m "Changes made on %date% at %time%"
@echo on
git push
@echo off
git pull