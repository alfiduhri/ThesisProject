-- example script for using "get_origin()"
-- prints the home and ekf origin lat long and altitude to the console every 5 seconds

waypoints_data = {} -- Define the table to store location data
home = ahrs:get_home()  -- Get the home data
origin = ahrs:get_origin()  -- Get the origin data
local index = 0

function save_location(location,index)
	--local rtl_checking = script_rtl:get()
	local file = io.open("location_log.txt", "a")  -- Open file in append mode
	
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
		table.insert(location_data,{index, curr, frame, cmd, p1, p2, p3, p4, lat, long, alt,auto})
        
		-- Save the data into log file
		file:write(string.format("%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%.6f\t%.6f\t%.2f\t%d\n",index, curr, frame, cmd, p1, p2, p3, p4, lat, long, alt,auto)) -- \t is <tab> 
		file:close()
	end
end
--]]

function update ()
	location = ahrs:get_location()
	
	if location then
        gcs:send_text(0, string.format("Location - Index:%d Lat:%.1f Long:%.1f Alt:%.1f",index, location:lat(), location:lng(), location:alt()))
		save_location(location,index)
		index = index+1
    end
	
    return update, 5000

end

return update(), 5000