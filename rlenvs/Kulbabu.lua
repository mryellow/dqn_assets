local classic = require 'classic'

local Kulbabu, super = classic.class('Kulbabu', Env)

-- Constructor
function Kulbabu:_init(opts)
  opts = opts or {}

  self.width = opts.width or 24
  self.height = opts.height or 24
  self.nChannels = opts.nChannels or 1
  self.screen = torch.Tensor(self.nChannels, self.height, self.width):zero()
end

function Kulbabu:getStateSpec()
  return {'real', {self.nChannels, self.height, self.width}, {0, 1}}
end

-- 1 action required, of type 'int', of dimensionality 1, between 0 and 2
function Kulbabu:getActionSpec()
  return {'int', 1, {0, 2}}
end

function Kulbabu:getDisplaySpec()
  return {'real', {self.nChannels, self.height, self.width}, {0, 1}}
end

-- Min and max reward
function Kulbabu:getRewardSpec()
  return 0, 1
end

-- Redraws screen based on state
function Kulbabu:clear()
  -- Reset screen
  self.screen:zero()
end

-- Starts new game
function Kulbabu:start()
  -- Clear screen
  self:clear()

  -- TODO: Subscribe to range sensor topics
  -- TODO: self.screen[{{1}, {self.size}, {self.player.x, self.player.x + self.player.width - 1}}] = 1

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
