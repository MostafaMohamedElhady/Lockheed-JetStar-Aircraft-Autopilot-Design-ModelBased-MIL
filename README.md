# Model-in-the-Loop Phase for Autopilot Design and Validation
<p align="center"> 
<img src="https://github.com/MostafaMohamedElhady/MostafaMohamedElhady/blob/main/Photos/FlightGear%20scene.png" alt="Autopilot Design Flowchart" width="1000" /> </a> </p>

## 📝 Project summary: 
This project demonstrates the model-in-the-loop development of an autopilot for the Lockheed JetStar aircraft, using MATLAB/Simulink with single-input-single-output (SISO) linear control theories to meet flying quality criteria. The designed controller is then validated on a linear multi-input, multi-output (MIMO) system and finally tested on a full nonlinear aircraft model to ensure robust performance across modeling complexities.

## 🚩 Project highlights: 
* Studying and reviewing mathematical modeling foundations for fixed-wing aircrafts.
* Building full non-linear model of the A/C for dynamics simulation using MATLAB & SIMULINK.
* Building full linear model for A/C for dynamics simulation using MATLAB & SIMULINK.
* Designing Longitudinal autopilot (Elevator from pitch, Throttle from airspeed and Altitude hold loops) for linearized aircraft model using single-input-single-output tool in MATLAB.
* Designing Lateral autopilot (Yaw damper, Roll controller and Heading controller loops) for linearized aircraft model using single-input-single-output tool in MATLAB.
* Tuning the designed controllers by iteratively verifying their performance on multi-input-multi-output linearized aircraft model.
* Testing the full autopilot (longitudinal + lateral controllers) on the nonlinear aircraft model to verify the performance of the controller designed using linear control techniques on a more accurate system.
* Performing a specified mission consisting of the following phases/manuevers: (1) Climb (2) Cruise (3) 360-degree turn (4) Simultaneous Climb and Turn manuever (5) Descent.

  
## 🧩 SIMULINK Model Architecture
<p align="center"> 
<img src="https://github.com/MostafaMohamedElhady/Lockhead-Jetstar-Autopilot/blob/main/flowchart4.svg" alt="Autopilot Design Flowchart" width="1000" /> </a> </p>

## ✈️ Mission profile and demonstration
The fully autonomous mission composed of the following stages requiring/commanding different attitudes or headings:
1. Climbing at altitude of 10,000 ft at 1500 ft/min climb rate.<br />
2. Cruising at constant velocity and heading angle for 2 minutes.<br />
3. Making a circle by commanding a change in the heading angle of 360 degrees.<br />
4. Cruising at constant velocity and heading angle for 2 minutes.<br />
5. Commanding a change in the heading angle with 30 degrees along with a step input of 100 ft in altitude.<br />
6. Descending down to the initial altitude by reducing the altitude by 10,000 ft at 1500 ft/min descent rate.<br />

** To see the full mission demonstration, please refer to: [Fully Autonomous Mission Demo (16x speed)](https://www.youtube.com/watch?v=Ek5aRqp5JpQ).<br />

## ▶️ How to run the project

Follow these steps to set up and run the project:

### 1. 📦 Clone the Repository

You have two options:

#### Using GitHub Desktop or GitKraken

- **GitHub Desktop:**
  1. Open GitHub Desktop and sign in if prompted.
  2. Go to **File > Clone Repository**.
  3. Select this repository, choose a destination folder, and click **Clone**.

- **GitKraken:**
  1. Open GitKraken and sign in if needed.
  2. Select **Clone a repo**, enter the repository URL, choose where to save it locally, and proceed.

#### Or, Clone via the Command Line
```
git clone https://github.com/username/repo-name.git
cd repo-name
```
### 2. 🚀 Run the Project

To open the project in MATLAB, enter the following command in the MATLAB command window:
```
uiopen('Project file path', 1)
```

Replace `'Project file path'` with the actual file (.prj file) path to your project's main file within the cloned directory.

**Tip:**  
Using GitHub Desktop or GitKraken offers a convenient visual interface for managing the project, while the command line offers speed and flexibility. Once cloned, ensure you have MATLAB installed and the path to your project file ready for a smooth start.

## 🧭 How to Navigate and Work with the Project
This project provides an interactive environment for exploring both the nonlinear and linear dynamic models of the Lockheed JetStar aircraft. You can analyze how modifications to specific aircraft parameters influence overall performance and stability.

Although the internal logic of the autopilot architecture is encrypted, you are able to:

- **View the complete system architecture:** Inspect the overall structure and signal flow of the autopilot within the model.

- **Modify mission command inputs:** Change the mission profiles or trajectory commands and observe how these affect the autopilot’s behavior.

- **Assess autopilot response:** Analyze how the designed autopilot system performs in various scenarios and profiles.

- **Learn from model organization:** Experience a well-structured Simulink model that makes use of hierarchical subsystems and efficient signal management using Simulink buses. This organizational approach helps you understand best practices in model layout and data flow.

By experimenting with the provided tools and features, you will gain hands-on insight into aircraft dynamics, the impact of parameter changes, and advanced model-based architecture for control systems. This makes the project especially valuable for study, experimentation, and practical exploration.

### 1. Data Inspector Visualization

- All simulation signals are logged to the Data Inspector.
    - Output states and input control actions from both the nonlinear and linear aircraft models are available.
    - Data can be viewed in both English and SI units.
    - The Data Inspector enables easy visualization, comparison, and analysis of results.
### 2. Editing Mission Commands

- You can customize the model's input commands for altitude and heading by editing or replacing the relevant signals used in the mission.
- This is accomplished through the **Signal Editor** block in the Commands block in the model, which allows you to import, add, or overwrite commands using a `.mat` file.
- The process is well documented in the MATLAB documentation, providing step-by-step guidance for customizing simulation inputs through the [Signal Editor block](https://www.mathworks.com/help/simulink/slref/signaleditorblock.html).

### 3. Interfacing SIMULINK with FlightGear

- If you want to visualize an entire mission in FlightGear, make sure you have established the proper interface between your Simulink model and FlightGear. For a clear, step-by-step walkthrough, it is recommended to watch this [demonstration video.](https://www.youtube.com/watch?v=f8tdTiuj5lo&t=1827s)


## 🐞 Reporting Bugs & Feedback

If you find a bug or have suggestions for improvement, please [open an issue](https://github.com/username/repo-name/issues) on GitHub.
---

Thank you for checking out this project!  
If you have any questions, suggestions, or feedback, feel free to open an issue or contact me.
