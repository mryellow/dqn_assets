ros = require 'ros'
local classic = require 'classic'

local Kulbabu, super = classic.class('Kulbabu', Env)

-- Constructor
function Kulbabu:_init(opts)
  opts = opts or {}

  self.width = opts.width or 8
  self.height = opts.height or 1
  self.channels = 1
  self.screen = torch.Tensor(self.channels, self.height, self.width):zero()

  self.ns = "kulbabu"
  self.range_topics = {
    "range_1l",
    "range_1r",
    "range_2l",
    "range_2r",
    "range_3l",
    "range_3r",
    "range_4l",
    "range_4r"
  }
  self.range_msg = "sensor_msgs/Range"

  self.subs = {}
  self.pubs = {}

  ros.init('kulbabu_dqn')

  spinner = ros.AsyncSpinner()
  spinner:start()

  self.nh = ros.NodeHandle()
end

function Kulbabu:getStateSpec()
  return {'real', {self.channels, self.height, self.width}, {0, 1}}
end

-- forward, left-forward, right-forward, left, right
function Kulbabu:getActionSpec()
  return {'int', 1, {0, 4}}
end

function Kulbabu:getDisplaySpec()
  return {'real', {self.channels, self.height, self.width}, {0, 1}}
end

-- Min and max reward
function Kulbabu:getRewardSpec()
  return 0, 1
end

-- Reset screen
function Kulbabu:clear()
  self.screen:zero()
end

-- Starts new game
function Kulbabu:start()
  -- Clear screen
  self:clear()

  -- TODO: Subscribe to range sensor topics
  for i, topic in ipairs(self.range_topics) do
    subscriber = self.nh:subscribe("/" .. self.ns .. i .. "/" .. topic, self.range_msg, 100)
    subscriber:registerCallback(function(msg, header)
      print('Header:')
      print(header)
      print('Message:')
      print(msg)
      -- TODO: self.screen[{{1}, {self.size}, {self.player.x, self.player.x + self.player.width - 1}}] = 1
    end)
    table.insert(self.subs, subscriber)
  end
  -- TODO: Subscribe to states for goal relative position

  -- Return observation
  return self.screen
end

-- Steps in a game
function Kulbabu:step(action)
  -- Reward is 0 by default
  local reward = 0

  -- TODO: Publish twists for actions
  if action == 1 then

  elseif action == 2 then

  end

  -- TODO: Calculate reward based on reaching goal
  -- Will need subscription to states and trig for relative position.

  -- TODO: Check terminal condition
  local terminal = false

  return reward, self.screen, terminal
end

-- Returns (RGB) display of screen
function Kulbabu:getDisplay()
  return torch.repeatTensor(self.screen, 3, 1, 1)
end

return Kulbabu
