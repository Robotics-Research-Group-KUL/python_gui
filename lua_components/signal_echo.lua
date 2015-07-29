
--to create it, copy-paste the code in the lua deployer file
--[[

depl:loadComponent("eventEcho", "OCL::LuaComponent")
--... and get references to them
eventEcho = depl:getPeer("eventEcho")
 -- load the Lua hooks
eventEcho:exec_file(rttros.find_rospack_roslua(python_gui).."/signal_echo/signal_echo.lua")
--configure and starts
eventEcho:configure()
eventEcho:start()

--depl:connectPeers("<name of the supervisor/fsm component>","eventEcho")
depl:connect("eventEcho.event_out","<name of the supervisor/fsm component>", rtt.Variable("ConnPolicy"))

-- in the end create a stream
depl:stream("eventEcho.event_in",rtt.provides("ros"):topic("/events"))
]]--

require("rttlib")
tc=rtt.getTC()
if tc:getName() == "lua" then
	depl=tc:getPeer("Deployer")	
elseif tc:getName() == "Deployer" then
	depl=tc
end
 

-- The Lua component starts its life in PreOperational, so
-- configureHook can be used to set stuff up.

local inport
local outport

function configureHook()
   inport = rtt.InputPort("std_msgs.String", "event_in")    -- global variable!
   outport = rtt.OutputPort("string", "event_out")    -- global variable!
   tc:addEventPort(inport)
   tc:addPort(outport)
   return true
end
 
-- all hooks are optional!
--function startHook() return true end
 
function updateHook()
   local fs, ev_in = inport:read()
   outport:write(ev_in.data)
end
 
-- Ports and properties are the only elements which are not
-- automatically cleaned up. This means this must be done manually for
-- long living components:
function cleanupHook()
   tc:removePort("event_in")
   inport:delete()
   tc:removePort("event_out")
   outport:delete()
end

