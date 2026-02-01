; ------------------------------------------------------------------------------
; InnoSetup Script for Delos10 Service
;
; Please run BUILD.BAT
;
;       See   Z:\Tools\InnoSetup
;       or   http://www.jrsoftware.org/
; ------------------------------------------------------------------------------
#define MYNAME               "Esp32Flasher"
#define MYCOMPANYSHORT       "Delos S.r.l."
#define MYCOMPANY            "Delos S.r.l., Torino (TO)"
#define MYAPPCOPYRIGHT       "(C) 2026 Delos S.r.l., Corso Lecce, 15 – 10145 Torino (Italy)"
#define MYCOPYRIGHT          "(C) Delos S.r.l., Torino (TO)"
#define MYAPPPUBLISHER       "https://www.delos-international.com/"

// Set to 1 to sign the installer
#define VARIANT "full"

#ifndef MYVERSION
; Thou shalt not use other methods than Build.cmd to create a distribution!
; If thou do, your version will be ...
#   define MYVERSION   "1.0.0"
#endif

#ifdef SIGN
  #define SIGN_SETUP 1
#else
  #define SIGN_SETUP 0
#endif

; ------------------------------------------------------------------------------

; The minimun version of the .Net Runtime required to run {MYNAME} 
#define dotNetMinimumVersion "4.8.1"

; ------------------------------------------------------------------------------
#include "VersioningScheme.iss"
; *** Now set TARGET_BUILD_STATE to one of the TARGET_xx constants from
;     VersioningScheme.iss.
; TARGET_RELEASE <- TARGET_RC_x <- TARGET_BETA_x  <- TARGET_PROTOTYPE_x 

; efine TARGET_BUILD_STATE          TARGET_PROTOTYPE_1
;#define TARGET_BUILD_STATE          TARGET_BETA_1
; #define TARGET_BUILD_STATE          TARGET_RC_2
#define TARGET_BUILD_STATE          TARGET_RELEASE

; ------------------------------------------------------------------------------
; Some helpers around our versioning scheme ...
#define TARGET_IS_RELEASE       (TARGET_BUILD_STATE==TARGET_RELEASE)
#define TARGET_IS_BETA          ((TARGET_BUILD_STATE>=TARGET_BETA_1)       && (TARGET_BUILD_STATE<=_MAX_BETA))
#define TARGET_IS_PROTOTYPE     ((TARGET_BUILD_STATE>=TARGET_PROTOTYPE_1)  && (TARGET_BUILD_STATE<=_MAX_PROTOTYPE))
#define TARGET_IS_RC            ((TARGET_BUILD_STATE>=TARGET_RC_1)         && (TARGET_BUILD_STATE<TARGET_RC_5))

#if TARGET_IS_RELEASE
#   define PREFIX_TEXT      ""
#   define PREFIX_NR        ""
#elif TARGET_IS_BETA
#   define PREFIX_TEXT      "-BETA"
#   define PREFIX_NR        Str(TARGET_BUILD_STATE)
#elif TARGET_IS_PROTOTYPE
#   define PREFIX_TEXT      "-PROTO"
#   define PREFIX_NR        Str(TARGET_BUILD_STATE-100)
#else
#   define PREFIX_TEXT      "-RC"
#   define PREFIX_NR        Str(TARGET_BUILD_STATE-1000)
#endif
; ------------------------------------------------------------------------------

#if VER < EncodeVer(6,0,2)
  #error A more recent version of Inno Setup is required to compile this script (InnoSetup 6.0.2 or newer)!
#endif

#define MYAPPID MYVERSION + PREFIX_TEXT + PREFIX_NR

; ******************************************************************************
[Setup]
AppName={#MYNAME}
ArchitecturesAllowed=x64
ArchitecturesInstallIn64BitMode=x64
#if TARGET_BUILD_STATE
    AppVersion={#MYVERSION}{#PREFIX_TEXT}{#PREFIX_NR}
    AppId={#MYNAME}{#MYVERSION}b{#TARGET_BUILD_STATE}
#else
    AppVersion={#MYVERSION}
    AppId={#MYNAME}{#MYVERSION}
#endif
AppVerName={#MYNAME} V{#MYAPPID}
AppCopyright={#MYAPPCOPYRIGHT}
AppPublisher={#MYCOMPANY}
AppPublisherURL={#MYAPPPUBLISHER}
;
VersionInfoVersion={#MYVERSION}
VersionInfoCompany={#MYCOMPANY}
VersionInfoCopyRight={#MYCOPYRIGHT}
VersionInfoDescription={#MYNAME} for Esp32Flasher integration.
VersionInfoProductName={#MYNAME}
VersionInfoProductVersion={#MYVERSION}
;
UsePreviousAppDir=no
UsePreviousGroup=no
;
DefaultDirName={pf}\Delos\{#MYNAME}
DisableDirPage=yes
DisableWelcomePage=no
DefaultGroupName={#MYCOMPANYSHORT}\{#MYNAME}\V{#MYAPPID}
;
OutputDir=..\Setup_{#VARIANT}
OutputBaseFileName={#MYNAME}_V{#MYAPPID}_{#VARIANT}
;
ChangesAssociations=yes
ShowLanguageDialog=yes
; only poweruser or admin can install software on 2000/XP/VISTA
; SDB[A36693E5]: App. must read package from HLKM (see below)
; PrivilegesRequired=poweruser

; Too restrictive, only IT could install then, bad for a few customers
PrivilegesRequired=admin
; PrivilegesRequired=none   // Worx not, no write permissions in HKLM
; Require Win7 or newer
MinVersion=0, 6.1
;
;SetupIconFile=CSMconfig.ico
;UninstallDisplayIcon={app}\csmconfig_remove.ico
UninstallDisplayIcon={app}\{#MYNAME}.exe
DisableProgramGroupPage=yes
Compression=lzma2
SolidCompression=yes
;
;WizardImageFile=C:\Users\User\source\DelosSetup\Extras\Loghi\Delos-setup-Wizard.png
WizardImageStretch=no
;WizardSmallImageFile=C:\Users\User\source\DelosSetup\Extras\Loghi\Delos-setup-logo.png
WizardStyle=modern

#if (SIGN_SETUP==1)
; 2019-08-19(fw): Use the Esp32Flasher signing tool, specified by the command line argument /S.
; $f: the quoted file name of the file to be signed
SignTool=Delos10sign $f
; And sign the uninstaller as well.

SignedUninstaller=yes
;
; On overload of our sign server, let it a little more time.
;
; 2019-12-10 (tw): We sign a 260MB (!) file.
; Sign.bat will copy that from local to the pickup-folder on Z:. Then the sign-server will
; do its job and then it will be copied back from  the drop-folder to local.
; This _is_ time consuming, especially when working via VPN from a home office ...
SignToolRetryDelay=50000
SignToolRetryCount=0
#endif

[Files]
; Executables
Source: "..\..\src\Esp32FlasherUI\bin\Release\net10.0-windows\win-x64\*"; DestDir: "{app}"; Flags: ignoreversion recursesubdirs

; Documentation
Source: "..\Documentation\Licence_eng.rtf"; DestDir: "{app}\Doc"; Flags: ignoreversion recursesubdirs
Source: "..\Example\Readme.txt"; DestDir: "{app}\Doc"; Flags: ignoreversion recursesubdirs


[Dirs]
;Name: "{app}\inst"; Flags: uninsneveruninstall
Name: "{commondocs}\{#MYCOMPANYSHORT}\{#MYNAME}"; Flags: uninsneveruninstall

[Languages]
Name: English; MessagesFile: compiler:Default.isl; LicenseFile: ..\Documentation\Licence_eng.rtf; InfoBeforeFile: ..\Example\Readme.txt;
;Name: Espanol; MessagesFile: compiler:Languages\Spanish.isl;
;Name: "Français"; MessagesFile: compiler:Languages\French.isl;
;Name: Italiano; MessagesFile: compiler:Languages\Italian.isl;

[Registry]

[Icons]
; Currently no needs for icons.
Name: "{commondesktop}\Esp32Flasher"; Filename: {app}\Esp32FlasherUI.exe; WorkingDir: {app};  Comment: {#SetupSetting("AppVerName")}; Check: WizardIsTaskSelected('desktopIcon');
Name: "{commonprograms}\Esp32Flasher"; Filename: {app}\Esp32FlasherUI.exe; WorkingDir: {app};  Comment: {#SetupSetting("AppVerName")}; Check: WizardIsTaskSelected('startMenu');

Name: "{group}\Readme"; Filename: {app}\Doc\Readme.txt; WorkingDir: {app}\Doc;
Name: "{group}\Licence_eng"; Filename: {app}\Doc\Licence_eng.rtf; WorkingDir: {app}\Doc;

[Tasks]
; Currently no needs for icons.
Name: desktopIcon; Description: {cm:CreateDesktopIcon};
Name: startMenu; Description: {cm:CreateStartMenu};

[CustomMessages]
English.CreateDesktopIcon=Create a desktop icon.
English.CreateStartMenu=Create start menu entries.

[Run]

[UninstallRun]

[UninstallDelete]
;Type: files; Name: "{app}\inst\*"
;Type: dirifempty; Name: "{app}\inst"
Type: dirifempty; Name: "{app}"

[Code]


