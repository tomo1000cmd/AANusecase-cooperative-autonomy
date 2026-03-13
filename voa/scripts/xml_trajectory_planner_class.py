from matplotlib import pyplot as plt
import numpy as np
import xml.etree.ElementTree as ET

Chaserbin_offset = 0.7
## @class trajectory_path_planner
#  @brief This class is responsible for planning the trajectory for a harvester and a chaser.
#
#  The class takes into account the sorted field and determines the trajectory for both the harvester and the chaser.
#  The trajectory is determined based on whether the section length is even or odd, and whether the harvesting is done from inside out or outside in.
#  The class also takes into account the side of harvesting and the number of sections done.
#  The trajectory for the chaser is determined based on the trajectory of the harvester.
class trajectory_path_planner:
    
    ## @brief Constructor for the trajectory_path_planner class.
    #  @param self The object pointer.
    #  @param filepath The path to the XML file containing the field data.
    def __init__(self, filepath):
         ## @brief The path to the XML file containing the field data.
        self.filepath           = filepath
        ## @brief A list to store the field data.
        self.field              = []
        ## @brief A list to store the sorted field data.
        self.sortedfield       = []
        ## @brief A list to store the harvester's trajectory.
        self.harvester_trajectory         = []
        ## @brief A list to store the chaser's trajectory.
        self.chaser_trajectory  = []
        ## @brief A list to store the chaserbin field data.
        self.chaserbin_field    = []
        ## @brief A list to store the chaserbin field data (edited).
        self.chaserbin_field2   = []


    ## The xml_reader imports the xml-file and extracts all coordinate values. The values are put inside a nested list. 
    # The xml_reader currently works with one field   
    ## @brief This function reads an XML file and extracts all coordinate values.
    #  @param self The object pointer.
    def xml_reader(self):
        
        
        tree                    = ET.parse(self.filepath)           
        document                = tree.getroot()                        
        
        field_index             = 0                                       
        count_section           = len(document.findall('./field[@name="'+str(field_index)+'"]/section'))


        # Importing all the coordinates per section
        for section_index in range(count_section):
            
            # The relative path from the document element to all the coordinates in a section
            xml_coordinate  = './field[@name="'+str(field_index)+'"]/*[@name="'+str(section_index)+'"]/coordinates'

            # Making a list out of all the 
            section         = [list(map(float, coord.text.split(','))) for coord in document.findall(xml_coordinate)]
            
            # Appending the sections
            self.field.append(section)
    
    
    ## The trajectory_sorter sorts the sections.
    # The right order of sections should be: 0,2,1,4,3,6,5,8,... to make sure the harvester and the chaserbin will 
    # not drive on a row that has not been harvested yet. If the index of the section is even, the section will be harvested inside out. 
    # If the section is odd, the section will be harvested outside in.
    ## @brief This function sorts the sections of the field.
    #  @param self The object pointer.
    def trajectory_sorter(self):
        
        

        count_sections = 2   

        # Checks if the field has an even amount of sections 
        if len(self.field) % 2 == 0:

            for k in range(len(self.field)):

                section = self.field[k]

                # Checks the index of the secion. 
                if k % 2 == 0:
                    
                    # Now it will first append the next even section and then the uneven section at the same iteration 'k'.
                    # This will make sure the sections are not sorted like this: 0,1,2,3,4,5...
                    self.sortedfield.append(section)
                    
                    # Section -1 cannot be added, so this must be prevented 
                    if len(self.sortedfield) == count_sections:

                        count_sections += 2
                        self.sortedfield.append(self.field[k-1])

            # The last section will not be appended with an even amount of sections, so it has to be appended seperately.
            self.sortedfield.append(self.field[len(self.field)-1])

        else:

            # Same algorithm, but without appending the last section seperately 
            for k in range(len(self.field)):

                section = self.field[k]

                if k % 2 == 0:

                    self.sortedfield.append(section)

                    if len(self.sortedfield) == count_sections:

                        count_sections += 2
                        self.sortedfield.append(self.field[k-1])

        
        # The rows of the chaserbin field should be slightly shorter, so there is enough room for switching rows
        for k in range(len(self.sortedfield)):

            chaserbin_section = self.sortedfield[k]

            for l in range(int(len(chaserbin_section)/2)):

                # New coordinates are made
                # A coordinate is defined as: section[coordinate][x/y-coordinate]
                x = chaserbin_section[2*l][0]+Chaserbin_offset
                y = chaserbin_section[2*l][1]
                x2 = chaserbin_section[2*l+1][0]-Chaserbin_offset
                y2 = chaserbin_section[2*l+1][1]
                
                # The new coordinates are appended
                self.chaserbin_field2.append([x,y])
                self.chaserbin_field2.append([x2,y2])
                
        # This list is used as a reference later on when switching lanes, because the coordinates of the edited list will 
        # cannot be used for comparison (they don't line up with the harvester coordinates)
        for section in self.sortedfield:
            self.chaserbin_field += section 


    ## @brief This function plans the trajectory for a harvester and a chaser.
    #
    #  The function takes into account the sorted field and determines the trajectory for both the harvester and the chaser.
    #  The trajectory is determined based on whether the section length is even or odd, and whether the harvesting is done from inside out or outside in.
    #  The function also takes into account the side of harvesting and the number of sections done.
    #  The trajectory for the chaser is determined based on the trajectory of the harvester.
    #
    #  @param self The object pointer.
    #  @return A tuple containing the harvester's trajectory and the chaser's trajectory.
    def trajectory_planner(self):

        harvester_sectionsdone = 2
        harvestside = 0
        
        for k in range(len(self.sortedfield)):

            section = self.sortedfield[k]

            # Choosing between even and odd section length (this is odd)
            if (len(section)/2) % 2 != 0:
                
                # Checking if current field is inside out or outside in (inside out)
                if harvester_sectionsdone != k:
                    
                    # Defining the starting point of the row
                    if harvestside % 2 == 0:
                        start_coordinate_row = (len(section)/2)-1

                    else:
                        start_coordinate_row = (len(section)/2)
                    
                    # Defining how many points/rows should be skipped 
                    skip_x_direction = 1
                    skip_y_direction = 2

                    # Appending the points to the correctly sorted list and then skipping points
                    for j in range(len(section)): 

                        for coordinate in range(len(section)):

                            if coordinate != start_coordinate_row:
                                continue

                            self.harvester_trajectory.append(section[coordinate])

                            if coordinate % 2 == 0:
                                coordinate += skip_x_direction
                                self.harvester_trajectory.append(section[coordinate])
                                coordinate -= skip_y_direction

                            else:
                                coordinate -= skip_x_direction
                                self.harvester_trajectory.append(section[coordinate])
                                coordinate += skip_y_direction

                            # Updating the skipping_factor and the startpoint of the row
                            skip_y_direction += 2
                            start_coordinate_row = coordinate

                    harvestside += 1   

                # outside in 
                else:
                    harvester_sectionsdone += 2
                    count_o_y = (len(section)-2)
                    start_coordinate_row = 0
                        
                    for j in range(int(len(section)/2)):

                        if start_coordinate_row % 2 == 0:
                            self.harvester_trajectory.append(section[start_coordinate_row])
                            start_coordinate_row += 1
                            self.harvester_trajectory.append(section[start_coordinate_row])
                            
                            start_coordinate_row += count_o_y
                            count_o_y -= 2

                        else:
                            self.harvester_trajectory.append(section[start_coordinate_row])
                            start_coordinate_row -= 1
                            self.harvester_trajectory.append(section[start_coordinate_row])
                            
                            start_coordinate_row -= count_o_y
                            count_o_y -= 2

                    harvestside += 1

            # Even section length 
            else:
                
                # Inside out
                if harvester_sectionsdone != k:

                    if harvestside % 2 == 0:
                        start_coordinate_row = (len(section)/2)

                    else:
                        start_coordinate_row = (len(section)/2)-1
                    
                    skip_x_direction = 1
                    skip_y_direction = 2

                    for j in range(len(section)):   

                        for coordinate in range(len(section)):

                            if coordinate == start_coordinate_row:
                                self.harvester_trajectory.append(section[coordinate])

                                if coordinate % 2 == 0:
                                    coordinate += skip_x_direction
                                    self.harvester_trajectory.append(section[coordinate])
                                    coordinate -= skip_y_direction

                                else:
                                    coordinate -= skip_x_direction
                                    self.harvester_trajectory.append(section[coordinate])
                                    coordinate += skip_y_direction

                                skip_y_direction += 2
                                start_coordinate_row = coordinate      

                # Outside in 
                else:
                    harvester_sectionsdone += 2
                    count_o_y = (len(section)-2)
                    
                    if harvestside % 2 == 0:
                        start_coordinate_row = len(section)-1

                    else:
                        start_coordinate_row = len(section)-1

                    for j in range(int(len(section)/2)):

                        if start_coordinate_row % 2 == 0:
                            self.harvester_trajectory.append(section[start_coordinate_row])
                            start_coordinate_row += 1
                            self.harvester_trajectory.append(section[start_coordinate_row])
                            
                            start_coordinate_row += count_o_y
                            count_o_y -= 2

                        else:
                            self.harvester_trajectory.append(section[start_coordinate_row])
                            start_coordinate_row -= 1
                            self.harvester_trajectory.append(section[start_coordinate_row])
                            
                            start_coordinate_row -= count_o_y
                            count_o_y -= 2
        
        
        # The chaserbin will have to follow another trajectory based on the trajectory of the harvester
        chaser_sectionsdone = 2
        side = 0
        count_coordinates = 0
 
        for k in range(len(self.sortedfield)):
            section = self.sortedfield[k]

            # Inside out
            if chaser_sectionsdone != k:

                for j in range(len(section)):

                    for coordinate in enumerate(self.chaserbin_field):
                        
                        # Checking if 
                        if self.harvester_trajectory[count_coordinates] == coordinate[1]:
                            
                            # Appending the correct point depending on the harvesing side
                            if side % 2 == 0:
                                self.chaser_trajectory.append(self.chaserbin_field2[coordinate[0]-2])

                            else:
                                self.chaser_trajectory.append(self.chaserbin_field2[coordinate[0]+2])
                                
                    if j % 2 == 1:
                        side += 1

                    # The counter has to keep counting, also outside these for loops 
                    count_coordinates += 1

            # Outside in 
            else:
                chaser_pointsdone = 0

                for j in range(len(section)):

                    for coordinate in enumerate(self.chaserbin_field):

                        if self.harvester_trajectory[count_coordinates] == coordinate[1]:

                            if chaser_pointsdone >= 4:

                                if side % 2 == 0:
                                    self.chaser_trajectory.append(self.chaserbin_field2[coordinate[0]-2])

                                else:
                                    self.chaser_trajectory.append(self.chaserbin_field2[coordinate[0]+2])

                            else: 

                                if chaser_sectionsdone < 3:

                                    if side % 2 == 0:
                                        self.chaser_trajectory.append(self.chaserbin_field2[coordinate[0]-2])

                                    else:
                                        self.chaser_trajectory.append(self.chaserbin_field2[coordinate[0]-(((len(section)*3)-2))])

                                    chaser_pointsdone += 1

                                else:
                                    if side % 2 == 0:
                                            self.chaser_trajectory.append(self.chaserbin_field2[coordinate[0]-2])

                                    else:
                                        self.chaser_trajectory.append(self.chaserbin_field2[coordinate[0]-(((len(section)*4)-2))])

                                    chaser_pointsdone += 1

                        
                    if j % 2 == 1:
                        side += 1

                    # Same counter
                    count_coordinates += 1 

                chaser_sectionsdone += 2      
        
        # Returning the sorted trajectories
        return self.harvester_trajectory, self.chaser_trajectory


    


