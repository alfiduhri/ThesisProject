-- Define the table with data
local data_table = {
    {0, 1, 3, 16, 0, 0, 0, 0, -35.363262, 149.165237, 583.90, 1},
    {1, 2, 3, 16, 0, 0, 0, 0, -35.363262, 149.165237, 584.01, 1},
    {2, 3, 3, 16, 0, 0, 0, 0, -35.363262, 149.165237, 583.93, 1},
    {3, 4, 3, 16, 0, 0, 0, 0, -35.363262, 149.165237, 584.14, 1},
    {4, 5, 3, 16, 0, 0, 0, 0, -35.363262, 149.165237, 584.16, 1},
    {5, 6, 3, 16, 0, 0, 0, 0, -35.363262, 149.165237, 584.18, 1}
}

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
	file:write("QCG WPL 110\n")  -- Writing txt header

    -- Iterate through the table and write its contents
    for _, row in ipairs(table_data) do
        file:write(table.concat(row, "\t") .. "\n")  -- Join the row values with tabs and add a newline
    end

    file:close()  -- Close the file
	
	-- Delete the table data (Garbage collection)
	table_data = nil
end


-- Change the value of the first column to 100
sorting_table(data_table)

-- Save the table to "output.txt"
save_table_to_file("output.txt", data_table)