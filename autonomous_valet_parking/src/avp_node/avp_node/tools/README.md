1. Launch Planning sim

2. Run: ros2 topic echo /planning/mission_planning/goal

3. Go to RViz and publish goal pose in each spot in incremental order.
4. Go back to ros2 topic output and u will see all messages at once in bulk. Copy full output down to last triple dash.
5. Paste into script: python3 generate_parking_spot_locations.py


## Drop OFF

1. Load Lanelet2 Map using Planning Sim
![image](https://github.com/user-attachments/assets/d202483f-d297-4899-ae91-22df5f2a7019)



2.Set 2D Pose in first boundary of drop off.
![image](https://github.com/user-attachments/assets/a9ce85da-f4e7-4c9f-8ef5-a29aabf1c0af)

3. Run this command: ros2 topic echo /localization/kinematic_state
4. Termiante the terminal instantly running the ros2 command as it iwll keep printing the same message.
5. Look at the position pose part.

![image](https://github.com/user-attachments/assets/62b9e5cb-38a3-46ed-973f-a2721646d128)

That is ur first coordinate. the x y.

6. type "Clear" in terminal.

Do step 2 - 6 again. Get all 4 coordinates.

![image](https://github.com/user-attachments/assets/b499da81-dfb3-4e93-9875-25142329f6aa)

![image](https://github.com/user-attachments/assets/31b44111-ee97-47f6-8c51-132180bb6ca0)




![image](https://github.com/user-attachments/assets/2c25aade-8694-4895-8462-c98730884d7f)

7. edit the drop off code in the avp node file.

![image](https://github.com/user-attachments/assets/9e3afe2b-f9cc-4269-bdd3-3c2be3632b17)

8. rebuild and run. Test to see if queueing works and allllllllllllat
