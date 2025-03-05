-- example script for using "get_origin()"
-- prints the home and ekf origin lat long and altitude to the console every 5 seconds

waypoints_data = {} -- Define the table to store location data
home = ahrs:get_home()  -- Get the home data
origin = ahrs:get_origin()  -- Get the origin data
local index = 0

-- Funcation to save the location into the table with specified format
function save_location(location,index)
	if location then
		
		-- Waypoints format: (12 items)
		-- <INDEX> <CURRENT WP> <COORD FRAME> <COMMAND> <PARAM1> <PARAM2> <PARAM3> <PARAM4> 
		-- <PARAM5/X/LATITUDE> <PARAM6/Y/LONGITUDE> <PARAM7/Z/ALTITUDE> <AUTOCONTINUE>
		local lat = location:lat()*1e-7
		local long = location:lng()*1e-7
		local alt = location:alt()*1e-2
		local curr = 0
		local p1=0 
		local p2=0 
		local p3=0 
		local p4=0
		local frame = 3 -- 0 means absolute, 3 means relative
		local cmd = 16 -- 16 means navigate to waypoints
		local auto = 1 --autocontinue
		
		-- Print the stored data for confirmation
		gcs:send_text(0, string.format("Saved: Index = %d, Lat=%.6f, Lng=%.6f, Alt=%.2f m", index,lat, long, alt))
		
        -- Insert rows into the table
		table.insert(waypoints_data,{index, curr, frame, cmd, p1, p2, p3, p4, lat, long, alt,auto})
	end
end

-- Function to sort the data and change the value of the first column
function sorting_table(table_data)
    -- Sorting function (Descending order based on first column)
    table.sort(table_data, function(a, b)
        return a[1] > b[1]  -- Sort by the first column in descending order
    end)
    -- Change the value of the first column
    for i, row in ipairs(table_data) do
        row[1] = i-1  -- Change the value of the first column (index 1) to new_value
    end
end

-- Function to save the table to a text file
function save_table_to_file(filename, table_data)
    local file = io.open(filename, "w") -- Open file for writing
	-- Creating header
	file:write("QGC WPL 110\n")  -- Writing txt header

    -- Iterate through the table and write its contents
    for _, row in ipairs(table_data) do
        file:write(table.concat(row, "\t") .. "\n")  -- Join the row values with tabs and add a newline
    end

    file:close()  -- Close the file
	
	-- Delete the table data (Garbage collection)
	table_data = nil
end

-- Function to load the mission
function read_mission(file_name)

    -- Open file
    file = assert(io.open(file_name), 'Could not open :' .. file_name)
  
    -- check header
    assert(string.find(file:read('l'),'QGC WPL 110') == 1, file_name .. ': incorrect format')
  
    -- clear any existing mission
    assert(mission:clear(), 'Could not clear current mission')
    
  
    -- read each line and write to mission
    local item = mavlink_mission_item_int_t()
    local index = 0
    local fail = false
    while true and not fail do
       local line = file:read()
       if not line then
          break
       end
       local ret, _, seq, _--[[ curr ]], frame, cmd, p1, p2, p3, p4, x, y, z, _--[[ autocont ]] = string.find(line, "^(%d+)%s+(%d+)%s+(%d+)%s+(%d+)%s+([-.%d]+)%s+([-.%d]+)%s+([-.%d]+)%s+([-.%d]+)%s+([-.%d]+)%s+([-.%d]+)%s+([-.%d]+)%s+(%d+)")
       if not ret then
          fail = true
          break
       end
       if tonumber(seq) ~= index then
          fail = true
          break
       end
       item:seq(tonumber(seq))
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
    if fail then
       mission:clear()  --clear anything already loaded
       error(string.format('failed to load mission at seq num %u', index))
    end
    gcs:send_text(0, string.format("Loaded %u mission items", index))
end

--Funcation to change the mode into auto mode
function auto_cmd()
	local param_scr = param:get('SCR_ENABLE')

	if param_scr==1 then
		gcs:send_text(0,'auto mode is activated')  
		vehicle:set_mode(10) --auto mode: 10 (from MAVLink)
	end
end

--Funcation to change the mode into loiter mode
function loiter_cmd()
    local mode = 12   -- Loiter mode is 12
	gcs:send_text(0,'loiter mode is activated')  
	vehicle:set_mode(mode)
end


-- Update function or main function
function update ()
	local param_user1 = param:get('SCR_USER1')

	if param_user1 == 0 then
		--Getting the location
		location = ahrs:get_location()
	
		--Save the location into the table called "waypoints_data"
		if location then
        	gcs:send_text(0, string.format("Location - Index:%d Lat:%.1f Long:%.1f Alt:%.1f",index, location:lat(), location:lng(), location:alt()))
			save_location(location,index)
			index = index+1
    	end

        return update, 10000
	elseif param_user1 == 1 --[[or <signal_lost>]]then 
		--Change the value of the first column to 0,1,2,...
		sorting_table(waypoints_data)

		--Save the table to "output.txt"
		save_table_to_file("location_log.txt", waypoints_data)

        -- Change into loiter command
        loiter_cmd()

		--Load the mission from .txt
		read_mission('location_log.txt')

		--Change into auto mode
    	auto_cmd()
	end

end

return update(), 5000