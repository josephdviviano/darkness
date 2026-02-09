import darkness

# TODO: We need to export service mask constants!
darknessroot = darkness.createRoot(darkness.services.SERVICE_ALL)

darknessroot.logToFile("darkness.log")
darknessroot.setLogLevel(4)

# Setup resources
darknessroot.loadResourceConfig("thief1.cfg")
darknessroot.loadConfigFile("darkness.cfg")

# Load the dype scripts (this should vary depending on the game type)
# darknessroot.loadDTypeScript("common.dtype", "General")
#darknessroot.loadDTypeScript("t1-types.dtype", "General")
#darknessroot.loadPLDefScript("t1-links.pldef", "General")
#darknessroot.loadPLDefScript("t1-props.pldef", "General")


# Bootstrapping finished. We can progress wit the actual engine run
darknessroot.bootstrapFinished();

# Service globals
lsrv = darkness.services.getLoopService()
isrv = darkness.services.getInputService()
# osrv = Services.ObjectService()

# ------------- State management -------------
# To sim menu switcher
# To be called from a key input handler
def switchToSimMenu():
    darkness.log_debug("Switching to sim menu")
    lsrv.requestLoopMode("GUIOnlyLoopMode")
    
# The reverse (to game switch from gui)
def switchToGameMode():
    darkness.log_debug("Switching to game mode")
    # TODO: Render service -> set world visible
    lsrv.requestLoopMode("AllClientsLoopMode")
    # guisrv.setActiveSheet(GameModeSheet)
    # guisrv.setVisible(true)
    # guisrv.setActive(false)
    # just to be sure
    isrv.setBindContext("game")


# ----- Input handlers -----
# Example: Handler to switch to sim menu (mapped on esc f.e.)
def simMenuRequest(msg):
    switchToSimMenu()

def exitRequest(msg):
#    log_info("Received a message for exitRequest " + str(msg.event))
    darkness.log_info("Termination requested!")
    lsrv.requestTermination()

def debugFrameRequest(msg):
    darkness.log_info("Received a message for frame debugger. Will debug one frame")
    darkness.log_info("Comand field: " + msg.command)
    darkness.log_info("Comand params: " + str(msg.params))
    lsrv.debugOneFrame()

# ------------ Main code -------------
def databaseProgressUpdate(msg):
    darkness.log_info("Loading progress : " + str(msg.completed  * 100))
#    loadScreen.setLeftGauge(msg['progress_master'])
#    loadScreen.setRightGauge(msg['progress_slave'])
#    rendersrv.renderOneFrame()
#
#    if (msg.finished()):
#	loadScreen.finished()
# 

# Setting bind context. This can be used to distingush between, for example, editor and game, etc.
isrv.createBindContext("game")
isrv.setBindContext("game")

# Callback on escape key trap
isrv.registerCommandTrap("exit_request", exitRequest)
# Debug frame shortcut trap
isrv.registerCommandTrap("debug_frame", debugFrameRequest)

# Map to escape and 1 keys (TODO: hardcoded_bind command could come handy to avoid breakage)
isrv.command("bind esc exit_request") # Would be sim_menu switcher binding, for example.
isrv.command("bind 1 debug_frame")
isrv.setInputMapped(True)

# A sample mission load
dbsrv = darkness.services.getDatabaseService()
dbsrv.setProgressListener(databaseProgressUpdate)
# dbsrv.load("miss1.mis", darkness.services.DBM_COMPLETE)

# some on-screen text
drawsrv = darkness.services.getDrawService()

dsht = drawsrv.createSheet("default")
drawsrv.setActiveSheet(dsht)
atl = drawsrv.createAtlas()
# load a font
drawsrv.setFontPalette(darkness.services.PT_PCX, "darkpal.pcx", "General")
#drawsrv.setFontPalette(darkness.services.PT_PCX, "amappal.pcx", "General")
fnt = drawsrv.loadFont(atl, "TEXTFONT.FON" , "General")
lab = drawsrv.createRenderedLabel(fnt)
lab.setLabel("A small text on screen")
lab.setPosition(0,0)
dsht.addDrawOperation(lab)


# Loop setup and execution
if (not lsrv.requestLoopMode("GUIOnlyLoopMode")):
    log_error("Could not set loop mode to GUIOnlyLoopMode!")
else:
    lsrv.run() # Run the main loop
    
# Termination
darkness.log_info("Terminating Darkness");

isrv.unregisterCommandTrap("debug_frame")
isrv.unregisterCommandTrap("exit_request")
dbsrv.unsetProgressListener()
