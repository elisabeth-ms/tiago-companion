import pandas as pd
import itertools
import random

# List of objects
objects = ['cereal1', 'cereal3', 'milk1', 'milk2', 'sugar1', 'sugar3', 'nutella', 'jam1', 'jam2', 'jam3', 'oil', 'tomato']

# Define orientations for each object type
orientations = {
    'cereal1': ['Up', 'Down', 'Up on Table', 'Down on Table', 'Tilted'],
    'cereal3': ['Up', 'Down', 'Up on Table', 'Down on Table', 'Tilted'],
    'sugar1': ['Up', 'Down', 'Up on Table', 'Down on Table', 'Tilted'],
    'milk1': ['Up', 'Up on Table', 'Down on Table', 'Tilted'],
    'milk2': ['Up', 'Up on Table', 'Down on Table', 'Tilted'],
    'sugar3': ['Up', 'Up on Table', 'Down on Table', 'Tilted'],
    'nutella': ['Up'],
    'jam1': ['Up'],
    'jam2': ['Up'],
    'jam3': ['Up'],
    'oil': ['Up'],
    'tomato': ['Up']
}

# Generate combinations of 5 objects
combinations = list(itertools.combinations(objects, 4))

# Calculate the base number of variations per combination
variations_per_combination = 2000 // len(combinations)

# Calculate the shortfall
shortfall = 2000 - (variations_per_combination * len(combinations))

# Create a dataframe
df = pd.DataFrame(combinations, columns=['Obj_A', 'Obj_B', 'Obj_C', 'Obj_D'])

# Expand each combination by varying occlusions and orientations
expanded_rows = []
case_num = 1
for idx, row in enumerate(df.iterrows()):
    variations = variations_per_combination + 1 if idx < shortfall else variations_per_combination
    for _ in range(variations):
        new_row = {}
        new_row['Case_Num'] = case_num
        for col in df.columns:
            new_row[col] = row[1][col]
            new_row[col + '_Occ'] = 'Visible' if random.random() > 0.7 else 'Partial'
            new_row[col + '_Ori'] = random.choice(orientations[row[1][col]])
            # Determine which object is occluded by another
            other_objects = [obj for obj in row[1] if obj != row[1][col]]
            new_row[col + '_Occ_By'] = random.choice(other_objects) if new_row[col + '_Occ'] == 'Partial' else 'None'
        expanded_rows.append(new_row)
        case_num += 1

# Convert expanded rows to a new dataframe
df_expanded = pd.DataFrame(expanded_rows)

# Reorder columns to have Case_Num first, then object names
cols = ['Case_Num'] + df.columns.tolist() + [col + suffix for col in df.columns for suffix in ['_Occ', '_Ori', '_Occ_By']]
df_expanded = df_expanded[cols]

# Save to Excel
df_expanded.to_excel('object_combinations_with_occlusions.xlsx', index=False)



