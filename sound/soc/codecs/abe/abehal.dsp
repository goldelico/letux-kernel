# Microsoft Developer Studio Project File - Name="ABEHAL" - Package Owner=<4>
# Microsoft Developer Studio Generated Build File, Format Version 6.00
# ** DO NOT EDIT **

# TARGTYPE "Win32 (x86) Console Application" 0x0103

CFG=ABEHAL - Win32 Debug
!MESSAGE This is not a valid makefile. To build this project using NMAKE,
!MESSAGE use the Export Makefile command and run
!MESSAGE
!MESSAGE NMAKE /f "ABEHAL.mak".
!MESSAGE
!MESSAGE You can specify a configuration when running NMAKE
!MESSAGE by defining the macro CFG on the command line. For example:
!MESSAGE
!MESSAGE NMAKE /f "ABEHAL.mak" CFG="ABEHAL - Win32 Debug"
!MESSAGE
!MESSAGE Possible choices for configuration are:
!MESSAGE
!MESSAGE "ABEHAL - Win32 Release" (based on "Win32 (x86) Console Application")
!MESSAGE "ABEHAL - Win32 Debug" (based on "Win32 (x86) Console Application")
!MESSAGE

# Begin Project
# PROP AllowPerConfigDependencies 0
# PROP Scc_ProjName "ABEHAL"
# PROP Scc_LocalPath "m:\a0918484_L1doc\ABE_Firmware\HAL\src"
CPP=cl.exe
RSC=rc.exe

!IF  "$(CFG)" == "ABEHAL - Win32 Release"

# PROP BASE Use_MFC 0
# PROP BASE Use_Debug_Libraries 0
# PROP BASE Output_Dir "Release"
# PROP BASE Intermediate_Dir "Release"
# PROP BASE Target_Dir ""
# PROP Use_MFC 0
# PROP Use_Debug_Libraries 0
# PROP Output_Dir "Release"
# PROP Intermediate_Dir "Release"
# PROP Ignore_Export_Lib 0
# PROP Target_Dir ""
# ADD BASE CPP /nologo /W3 /GX /O2 /D "WIN32" /D "NDEBUG" /D "_CONSOLE" /D "_MBCS" /YX /FD /c
# ADD CPP /nologo /W3 /GX /O2 /D "WIN32" /D "NDEBUG" /D "_CONSOLE" /D "_MBCS" /YX /FD /c
# ADD BASE RSC /l 0x3009 /d "NDEBUG"
# ADD RSC /l 0x3009 /d "NDEBUG"
BSC32=bscmake.exe
# ADD BASE BSC32 /nologo
# ADD BSC32 /nologo
LINK32=link.exe
# ADD BASE LINK32 kernel32.lib user32.lib gdi32.lib winspool.lib comdlg32.lib advapi32.lib shell32.lib ole32.lib oleaut32.lib uuid.lib odbc32.lib odbccp32.lib kernel32.lib user32.lib gdi32.lib winspool.lib comdlg32.lib advapi32.lib shell32.lib ole32.lib oleaut32.lib uuid.lib odbc32.lib odbccp32.lib /nologo /subsystem:console /machine:I386
# ADD LINK32 kernel32.lib user32.lib gdi32.lib winspool.lib comdlg32.lib advapi32.lib shell32.lib ole32.lib oleaut32.lib uuid.lib odbc32.lib odbccp32.lib kernel32.lib user32.lib gdi32.lib winspool.lib comdlg32.lib advapi32.lib shell32.lib ole32.lib oleaut32.lib uuid.lib odbc32.lib odbccp32.lib /nologo /subsystem:console /machine:I386

!ELSEIF  "$(CFG)" == "ABEHAL - Win32 Debug"

# PROP BASE Use_MFC 0
# PROP BASE Use_Debug_Libraries 1
# PROP BASE Output_Dir "Debug"
# PROP BASE Intermediate_Dir "Debug"
# PROP BASE Target_Dir ""
# PROP Use_MFC 0
# PROP Use_Debug_Libraries 1
# PROP Output_Dir "Debug"
# PROP Intermediate_Dir "Debug"
# PROP Ignore_Export_Lib 0
# PROP Target_Dir ""
# ADD BASE CPP /nologo /W3 /Gm /GX /ZI /Od /D "WIN32" /D "_DEBUG" /D "_CONSOLE" /D "_MBCS" /YX /FD /GZ /c
# ADD CPP /nologo /W4 /Gm /GX /ZI /Od /D "WIN32" /D "_DEBUG" /D "_CONSOLE" /D "_MBCS" /FR /YX /FD /GZ /c
# ADD BASE RSC /l 0x3009 /d "_DEBUG"
# ADD RSC /l 0x3009 /d "_DEBUG"
BSC32=bscmake.exe
# ADD BASE BSC32 /nologo
# ADD BSC32 /nologo
LINK32=link.exe
# ADD BASE LINK32 kernel32.lib user32.lib gdi32.lib winspool.lib comdlg32.lib advapi32.lib shell32.lib ole32.lib oleaut32.lib uuid.lib odbc32.lib odbccp32.lib kernel32.lib user32.lib gdi32.lib winspool.lib comdlg32.lib advapi32.lib shell32.lib ole32.lib oleaut32.lib uuid.lib odbc32.lib odbccp32.lib /nologo /subsystem:console /debug /machine:I386 /pdbtype:sept
# ADD LINK32 kernel32.lib user32.lib gdi32.lib winspool.lib comdlg32.lib advapi32.lib shell32.lib ole32.lib oleaut32.lib uuid.lib odbc32.lib odbccp32.lib kernel32.lib user32.lib gdi32.lib winspool.lib comdlg32.lib advapi32.lib shell32.lib ole32.lib oleaut32.lib uuid.lib odbc32.lib odbccp32.lib /nologo /subsystem:console /debug /machine:I386 /pdbtype:sept

!ENDIF

# Begin Target

# Name "ABEHAL - Win32 Release"
# Name "ABEHAL - Win32 Debug"
# Begin Group "Source Files"

# PROP Default_Filter "cpp;c;cxx;rc;def;r;odl;idl;hpj;bat"
# Begin Source File

SOURCE=.\ABE_API.c
# End Source File
# Begin Source File

SOURCE=.\ABE_DBG.c
# End Source File
# Begin Source File

SOURCE=.\ABE_EXT.c
# End Source File
# Begin Source File

SOURCE=.\ABE_INI.c
# End Source File
# Begin Source File

SOURCE=.\ABE_IRQ.c
# End Source File
# Begin Source File

SOURCE=.\ABE_LIB.c
# End Source File
# Begin Source File

SOURCE=.\ABE_MAIN.c
# End Source File
# Begin Source File

SOURCE=.\ABE_SEQ.c
# End Source File
# Begin Source File

SOURCE=.\ABE_TEST.C
# End Source File
# End Group
# Begin Group "Header Files"

# PROP Default_Filter "h;hpp;hxx;hm;inl"
# Begin Source File

SOURCE=.\ABE_API.H
# End Source File
# Begin Source File

SOURCE=.\ABE_CM_ADDR.h
# End Source File
# Begin Source File

SOURCE=.\ABE_COF.H
# End Source File
# Begin Source File

SOURCE=.\ABE_DAT.H
# End Source File
# Begin Source File

SOURCE=.\ABE_DBG.H
# End Source File
# Begin Source File

SOURCE=.\ABE_DEF.H
# End Source File
# Begin Source File

SOURCE=.\ABE_define.h
# End Source File
# Begin Source File

SOURCE=.\ABE_DM_ADDR.h
# End Source File
# Begin Source File

SOURCE=.\ABE_EXT.h
# End Source File
# Begin Source File

SOURCE=.\ABE_functionsId.h
# End Source File
# Begin Source File

SOURCE=.\ABE_FW.H
# End Source File
# Begin Source File

SOURCE=.\ABE_INITxxx_labels.h
# End Source File
# Begin Source File

SOURCE=.\ABE_LIB.H
# End Source File
# Begin Source File

SOURCE=.\ABE_MAIN.H
# End Source File
# Begin Source File

SOURCE=.\ABE_REF.H
# End Source File
# Begin Source File

SOURCE=.\ABE_SEQ.H
# End Source File
# Begin Source File

SOURCE=.\ABE_SM_ADDR.h
# End Source File
# Begin Source File

SOURCE=.\ABE_SYS.H
# End Source File
# Begin Source File

SOURCE=.\ABE_taskId.h
# End Source File
# Begin Source File

SOURCE=.\ABE_TEST.h
# End Source File
# Begin Source File

SOURCE=.\ABE_TYP.H
# End Source File
# Begin Source File

SOURCE=.\ABE_typedef.h
# End Source File
# Begin Source File

SOURCE=.\C_ABE_FW.CM
# End Source File
# Begin Source File

SOURCE=.\C_ABE_FW.lDM
# End Source File
# Begin Source File

SOURCE=.\C_ABE_FW.PM
# End Source File
# Begin Source File

SOURCE=.\C_ABE_FW.SM32
# End Source File
# Begin Source File

SOURCE=.\CodingStyle.txt
# End Source File
# End Group
# Begin Group "Resource Files"

# PROP Default_Filter "ico;cur;bmp;dlg;rc2;rct;bin;rgs;gif;jpg;jpeg;jpe"
# End Group
# End Target
# End Project
