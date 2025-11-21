# Quick Start Guide - Industrial Robotics Guided Course

## Getting Started in 5 Minutes

### 1. Check Python Installation

```bash
python3 --version  # Should be Python 3.8 or higher
```

### 2. Create Virtual Environment

```bash
# Navigate to course directory
cd indus40_rob

# Create virtual environment
python3 -m venv venv

# Activate it
source venv/bin/activate  # Linux/Mac
# or
venv\Scripts\activate  # Windows
```

### 3. Install Required Packages

```bash
pip install --upgrade pip
pip install numpy matplotlib scipy
pip install roboticstoolbox-python spatialmath-python
pip install jupyter notebook
```

### 4. Launch Jupyter Notebook

```bash
jupyter notebook
```

Your browser should open automatically. If not, copy the URL from the terminal.

### 5. Start with TP 00

Open `tp_00_python_fundamentals.ipynb` and work through the Python fundamentals.

## Course Sequence

Follow this order:

1. **TP 00** (4h) - Python Fundamentals → Master NumPy and Matplotlib
2. **TP 01** (8h) - Geometric Transformations → Understand 2D/3D transforms
3. **TP 02** (8h) - Forward Kinematics → DH, MDH, Elementary methods
4. **TP 03** (8h) - Inverse Kinematics → Solve for joint angles
5. **TP 04** (8h) - Jacobian Analysis → Velocity and singularities
6. **TP 05** (8h) - Trajectory Planning → Motion control

**Total: 44 hours**

## Working Method for Each TP

### Step 1: Read Theory
Open the main notebook (e.g., `tp_02_fk_main.ipynb`) and read the introduction.

### Step 2: Hand-Written Solutions (1-2h)
- Grab paper and pencil
- Work through the problems analytically
- Draw diagrams and coordinate frames
- Derive equations step-by-step

### Step 3: Python Implementation (2-3h)
- Open the student notebook
- Implement your solutions in code
- Test with provided examples
- Debug and refine

### Step 4: Validation (30min-1h)
- Compare with Robotics Toolbox results
- Visualize your solutions
- Analyze any discrepancies
- Document findings

## Essential Commands

### NumPy Basics
```python
import numpy as np

# Create arrays
a = np.array([1, 2, 3])
M = np.array([[1, 0], [0, 1]])

# Matrix operations
M_inv = np.linalg.inv(M)  # Inverse
M_T = M.T  # Transpose
det = np.linalg.det(M)  # Determinant

# Trigonometry
angle = np.pi/4
cos_val = np.cos(angle)
sin_val = np.sin(angle)
```

### Matplotlib Basics
```python
import matplotlib.pyplot as plt

# 2D plot
plt.figure()
plt.plot([0, 1], [0, 1])
plt.xlabel('x')
plt.ylabel('y')
plt.grid(True)
plt.show()

# 3D plot
from mpl_toolkits.mplot3d import Axes3D
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.plot([0, 1], [0, 1], [0, 1])
plt.show()
```

### Robotics Toolbox Basics
```python
import roboticstoolbox as rtb
from spatialmath import SE3
import numpy as np

# Create a transformation
T = SE3.Rx(np.pi/2) * SE3.Trans(1, 0, 0)
print(T)

# Create a simple robot
robot = rtb.models.DH.Puma560()
print(robot)

# Forward kinematics
q = [0, 0, 0, 0, 0, 0]  # Joint angles
T = robot.fkine(q)
print(T)

# Plot robot
robot.plot(q)
```

## Tips for Success

### 📝 Before Each Session
- Review previous session's work
- Read theory in main notebook
- Prepare questions

### 💻 During Each Session
- Start with simple cases
- Test frequently
- Save work regularly
- Ask questions early

### ✅ After Each Session
- Complete any unfinished work
- Review solutions (if provided)
- Practice similar problems
- Prepare for next session

## Common Issues and Solutions

### Issue: Import errors
```python
# If you see: ModuleNotFoundError: No module named 'roboticstoolbox'
# Solution: Make sure virtual environment is activated
source venv/bin/activate  # then reinstall
pip install roboticstoolbox-python
```

### Issue: Matplotlib plots don't show
```python
# Add this at the top of notebook
%matplotlib notebook  # or
%matplotlib inline
```

### Issue: Matrix dimension mismatch
```python
# Always check shapes
print(A.shape)  # Should be (m, n)
print(B.shape)  # Should be (n, p) for A @ B
```

### Issue: Angle units (radians vs degrees)
```python
# NumPy uses radians!
import numpy as np
angle_deg = 45
angle_rad = np.deg2rad(angle_deg)  # Convert to radians
result = np.cos(angle_rad)  # Correct

# Wrong:
# result = np.cos(angle_deg)  # This treats 45 as radians!
```

## Keyboard Shortcuts in Jupyter

- `Shift + Enter`: Run cell and move to next
- `Ctrl + Enter`: Run cell and stay
- `A`: Insert cell above
- `B`: Insert cell below
- `DD`: Delete cell
- `M`: Convert to Markdown
- `Y`: Convert to Code
- `Esc`: Exit edit mode
- `Enter`: Enter edit mode

## Getting Help

### In Jupyter Notebook
```python
# Get help on function
help(np.dot)
# or
?np.dot

# See available methods
np.  # then press Tab
```

### Online Resources
- NumPy Docs: https://numpy.org/doc/
- Matplotlib Gallery: https://matplotlib.org/stable/gallery/
- Robotics Toolbox: https://petercorke.github.io/robotics-toolbox-python/

### Ask Instructor
- During sessions
- Office hours
- Email with specific questions
- Course forum

## File Organization

Keep your work organized:

```
indus40_rob/
├── venv/                    # Virtual environment
├── tp_00_python_fundamentals.ipynb
├── tp_01.ipynb             # Student version
├── tp_01_sol.ipynb         # Solutions (after deadline)
├── tp_02_fk_main.ipynb     # Theory/overview
├── tp_02_fk_session1.ipynb # Session 1 work
├── tp_02_fk_session2.ipynb # Session 2 work
├── my_notes/               # Your notes (create folder)
│   ├── tp01_notes.pdf
│   ├── tp02_handwritten.pdf
│   └── ...
└── README.md               # This guide
```

## Submission Guidelines

### What to Submit
1. **Completed notebooks** with your code
2. **Hand-written solutions** (scanned PDF)
3. **Results and analysis** in notebook cells
4. **README** explaining your approach (optional but recommended)

### Format
- Notebooks: `.ipynb` files
- Hand-written: PDF scans (clear, readable)
- Name files: `TP0X_YourName.ipynb`

### Deadline
- Check course calendar
- Submit via course platform
- Late penalties may apply

## Study Group Recommendations

Form study groups of 2-4 students:
- Discuss approaches (before implementation)
- Debug together
- Explain concepts to each other
- Compare results

**Important:** Write your own code! Collaboration ≠ Copying

## Time Management

### Typical 4-hour Session
- 0:00-0:30 → Review theory
- 0:30-1:30 → Hand-written work
- 1:30-3:30 → Python implementation
- 3:30-4:00 → Validation & wrap-up

### Between Sessions (2-4 hours)
- Complete unfinished work
- Review and study
- Prepare questions
- Practice extra problems

## Assessment Preparation

### For Continuous Assessment
- Keep all hand-written work
- Document your code well
- Show validation results
- Explain your reasoning

### For Final Exam
- Review all TPs
- Practice similar problems
- Understand concepts, not just memorize
- Be able to explain your code

## What's Next?

After completing this guided course (44h), you'll be ready for:

**Industry 4.0 SAE Project (52h)**
- Apply everything you learned
- Work on realistic industrial project
- Team-based development
- See `indus40_rob_sae/` folder

## Quick Reference Sheet

### Homogeneous Transformation (2D)
```python
def Trans2D(x, y):
    return np.array([[1, 0, x],
                     [0, 1, y],
                     [0, 0, 1]])

def Rot2D(theta):
    c, s = np.cos(theta), np.sin(theta)
    return np.array([[c, -s, 0],
                     [s,  c, 0],
                     [0,  0, 1]])
```

### Homogeneous Transformation (3D)
```python
def TransX(d):
    return np.array([[1, 0, 0, d],
                     [0, 1, 0, 0],
                     [0, 0, 1, 0],
                     [0, 0, 0, 1]])

def RotX(theta):
    c, s = np.cos(theta), np.sin(theta)
    return np.array([[1, 0,  0, 0],
                     [0, c, -s, 0],
                     [0, s,  c, 0],
                     [0, 0,  0, 1]])
```

### DH Transformation
```python
def DH_matrix(theta, d, a, alpha):
    ct, st = np.cos(theta), np.sin(theta)
    ca, sa = np.cos(alpha), np.sin(alpha)
    return np.array([[ct, -st*ca,  st*sa, a*ct],
                     [st,  ct*ca, -ct*sa, a*st],
                     [0,   sa,     ca,    d   ],
                     [0,   0,      0,     1   ]])
```

---

**Ready to start?** Open `tp_00_python_fundamentals.ipynb` and begin your robotics journey!

**Questions?** Don't hesitate to ask during sessions or office hours.

**Good luck! 🤖**
