;****************************************************************************
; @def    CONFIG_BETA
;
; @brief  A macro to handle RELEASE / RC / BETA / PROTOTYPE targets.
;
; CONFIG_BETA must be one of the TARGET_xxxx #defines.
;
; The title in the main window will be decorated accordingly and we have
; several defines to mark code that exists only in non TARGET_RELEASE versions
; of the program.
; 
;  - @b TARGET_RELEASE @n
;    This is what ships to the customer
;  - @b TARGET_RC_xx @n
;    Feature freeze after BETA phase. Only bug-fixes should come in now !!
;  - @b TARGET_BETA_xx @n
;    We can have several BETA stages that will be published at least
;    inside the company (testing, demonstration) and may even make it to some customers
;    when some foolish salesman decides that this may be helpful.@n
;    Each BETA is supposed to accumulate all features of their predecessors plus some
;    additional fluff.
;  - @b TARGET_PROTO_xx @n
;    Prototype for a specific feature.@n
;    Prototypes are not necessarily synced with the current development (BETAs). 
;    They should address exactly one specific feature.
;
; We use #defines rather than const xxx because this is handled by preprocessor
; conditionals all over the source.
;
; @sa CMainFrame::OnUpdateFrameTitle
;****************************************************************************

; We compile a release that will ship to real customers
#define TARGET_RELEASE          0                   

; We compile a BETA version
#define TARGET_BETA_1           1                   
#define TARGET_BETA_2           2 
#define TARGET_BETA_3           3 
#define TARGET_BETA_4           4 
#define TARGET_BETA_5           5 
#define TARGET_BETA_6           6 
#define TARGET_BETA_7           7 
#define TARGET_BETA_8           8 
#define TARGET_BETA_9           9 
#define TARGET_BETA_10          10
#define TARGET_BETA_11          11
#define TARGET_BETA_12          12
#define TARGET_BETA_13          13
#define TARGET_BETA_14          14
#define TARGET_BETA_15          15
#define _MAX_BETA               TARGET_BETA_15

; We compile a prototype for some feature
#define TARGET_PROTOTYPE_1      101                 
#define TARGET_PROTOTYPE_2      102 
#define TARGET_PROTOTYPE_3      103 
#define TARGET_PROTOTYPE_4      104 
#define TARGET_PROTOTYPE_5      105 
#define TARGET_PROTOTYPE_6      106 
#define TARGET_PROTOTYPE_7      107 
#define TARGET_PROTOTYPE_8      108 
#define TARGET_PROTOTYPE_9      109 
#define TARGET_PROTOTYPE_10     110
#define _MAX_PROTOTYPE          TARGET_PROTOTYPE_10

; We compile a release candidate
#define TARGET_RC_1             1001                
#define TARGET_RC_2             1002
#define TARGET_RC_3             1003
#define TARGET_RC_4             1004
#define TARGET_RC_5             1005
#define _MAX_RC                 TARGET_RC_5




