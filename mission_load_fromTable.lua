-- Define the table with data
local data_table = {
    {0, 0,	0,	16,	0.000000,	0.000000,	0.000000,	0.000000,	-35.363262,	149.165237,	584.089966,	1},
    {1,	0,	3,	22,	0.000000,	0.000000,	0.000000,	0.000000,	-35.353105,	149.164049,	20.000000,	1},
    {2,	0,	3,	16,	0.000000,	0.000000,	0.000000,	0.000000,	-35.361606,	149.164907,	100.000000,	1},
    {3,	0,	3,	16,	0.000000,	0.000000,	0.000000,	0.000000,	-35.360202,	149.163585,	100.000000,	1},
    {4,	0,	3,	16,	0.000000,	0.000000,	0.000000,	0.000000,	-35.358830,	149.163425,	100.000000,	1},
    {5,	0,	3,	16,	0.000000,	0.000000,	0.000000,	0.000000,	-35.358073,	149.162140,	100.000000,	1},
    {6,	0,	3,	16,	0.000000,	0.000000,	0.000000,	0.000000,	-35.357645,	149.160453,	100.000000,	1}
}

function read_mission(table_name)

    -- clear any existing mission
    assert(mission:clear(), 'Could not clear current mission')
    
    -- read each line and write to mission
    local item = mavlink_mission_item_int_t()
    local index = 0
    for _,row in ipairs(table_name) do
        -- Waypoints format: (12 items)
		-- <INDEX> <CURRENT WP> <COORD FRAME> <COMMAND> <PARAM1> <PARAM2> <PARAM3> <PARAM4> 
		-- <PARAM5/X/LATITUDE> <PARAM6/Y/LONGITUDE> <PARAM7/Z/ALTITUDE> <AUTOCONTINUE>
        local seq,current,frame,cmd,p1,p2,p3,p4,x,y,z,auto = table.unpack(row)
       item:seq(tonumber(seq))
       item:current(tonumber(current))
       item:frame(tonumber(frame))
       item:command(tonumber(cmd))
       item:param1(tonumber(p1))
       item:param2(tonumber(p2))
       item:param3(tonumber(p3))
       item:param4(tonumber(p4))
       if mission:cmd_has_location(tonumber(cmd)) then
          item:x(math.floor(tonumber(x)*10^7))
          item:y(math.floor(tonumber(y)*10^7))
       else
          item:x(math.floor(tonumber(x)))
          item:y(math.floor(tonumber(y)))
       end
       item:z(tonumber(z))
       if not mission:set_item(index,item) then
          mission:clear() -- clear part loaded mission
          fail = true
          break
       end
       index = index + 1
    end
    gcs:send_text(0, string.format("Loaded %u mission items", index))
  end
  
  function auto_cmd()
      local param_scr = param:get('SCR_ENABLE')
  
      if param_scr==1 then
          gcs:send_text(0,'set_mode is active')  
          vehicle:set_mode(10) --auto mode: 10 (from MAVLink)
          --else
          --vehicle:set_mode(0)
      end
          
      --return auto_cmd, 1000
  end
  
  
  
  function update()
    read_mission('way.txt')
    auto_cmd()
  end
  
  return update, 5000