# Industrial Robotics 4.0 - Guided Course

## Course Overview

**Course Code:** GEII - Industrial Robotics  
**Level:** 3rd Year Engineering (GEII)  
**Language:** English  
**Total Hours:** 44 hours (practical sessions)  
**Prerequisites:** Basic Python programming, linear algebra, trigonometry

## Course Description

This guided course provides a comprehensive introduction to industrial robotics with a focus on mathematical modeling, kinematic analysis, and trajectory control. Students will develop both theoretical understanding and practical implementation skills using Python and the Robotics Toolbox for Python (Peter Corke).

## Learning Objectives

By the end of this course, students will be able to:

1. **Mathematical Foundations**
   - Master NumPy and Matplotlib for robotics applications
   - Implement homogeneous transformations
   - Work with rotation matrices and transformation chains

2. **Forward Kinematics**
   - Understand and apply Denavit-Hartenberg (DH) convention
   - Apply Modified DH (MDH) parameters
   - Use Elementary Transformation System
   - Validate solutions using Robotics Toolbox

3. **Inverse Kinematics**
   - Solve analytical IK for common robot configurations
   - Implement geometric and algebraic methods
   - Handle multiple solutions and workspace limits

## Course Structure

### TP 00: Python Fundamentals for Robotics (4h)
**File:** `tp_00_python_fundamentals.ipynb`

Essential NumPy and Matplotlib skills needed for the course:
- Array operations and broadcasting
- Linear algebra with NumPy
- 2D and 3D visualization with Matplotlib
- Matrix transformations
- Practical robotics examples

**Topics Covered:**
- NumPy arrays, indexing, slicing
- Matrix operations (dot, cross, transpose, inverse)
- Trigonometric functions
- 2D plotting (trajectories, vectors)
- 3D plotting (coordinate frames, robot configurations)
- Animation basics

### TP 01: Geometric Transformations (4h + 4h)
**Files:** 
- `tp_01.ipynb` (student version)
- `tp_01_sol.ipynb` (solutions with explanations)

Introduction to spatial transformations and homogeneous coordinates.

**Session 1 (4h):**
- 2D rotations and translations
- Homogeneous coordinates
- Composition of transformations
- Reference frame transformations

**Session 2 (4h):**
- 3D rotations (Roll-Pitch-Yaw, axis-angle)
- 3D homogeneous transformations
- Forward transformation chains
- Inverse transformations

**Deliverables:**
- Hand-written solutions to theoretical problems
- Python implementations
- Validation using Robotics Toolbox

### TP 02: Forward Kinematics (4h + 4h)
**Files:**
- `tp_02_fk_main.ipynb` (overview and theory)
- `tp_02_fk_session1.ipynb` (DH and MDH)
- `tp_02_fk_session2.ipynb` (Elementary transformations)

**Session 1 (4h):**
- Denavit-Hartenberg (DH) convention
- Modified DH (MDH) parameters
- 2-link planar manipulator
- 3-DOF cylindrical robot
- SCARA robot analysis

**Session 2 (4h):**
- Elementary Transformation System
- 6-DOF anthropomorphic manipulator
- Comparison: DH vs MDH vs Elementary
- Robotics Toolbox validation

**Robots Studied:**
- 2-link planar arm
- 3-DOF cylindrical robot
- SCARA robot
- PUMA 560
- UR5-like anthropomorphic manipulator

### TP 03: Inverse Kinematics (4h + 4h)
**Files:**
- `tp_03_ik_main.ipynb` (overview)
- `tp_03_ik_session1.ipynb` (analytical methods)
- `tp_03_ik_session2.ipynb` (advanced topics)

**Session 1 (4h):**
- Geometric approach for 2R and 3R planar robots
- Algebraic method for SCARA
- Multiple solutions analysis
- Workspace and reachability

**Session 2 (4h):**
- 6-DOF anthropomorphic arm IK
- Wrist partitioning method
- Singularities and solution selection
- Validation with Robotics Toolbox

**Key Concepts:**
- Closed-form solutions
- Elbow-up/elbow-down configurations
- Wrist center approach
- Joint limits handling

## Required Tools and Libraries

### Software Installation

```bash
# Create virtual environment
python3 -m venv robotics_env
source robotics_env/bin/activate  # Linux/Mac
# or
robotics_env\Scripts\activate  # Windows

# Install required packages
pip install numpy matplotlib scipy
pip install roboticstoolbox-python
pip install spatialmath-python
pip install ansitable
```

### Python Libraries Used

- **NumPy:** Numerical computations, matrix operations
- **Matplotlib:** 2D and 3D plotting, animations
- **SciPy:** Optimization and numerical methods
- **Robotics Toolbox:** Robot modeling and visualization
- **Spatial Math:** Rotation representations and transformations

## Working Methodology

Each practical session follows this structure:

1. **Theory Review (30 min)**
   - Key concepts presentation
   - Mathematical formulation
   - Algorithm description

2. **Hand-Written Exercises (1h)**
   - Solve problems on paper
   - Develop mathematical solutions
   - Sketch robot configurations

3. **Python Implementation (2h)**
   - Implement algorithms from scratch
   - Test with different robot configurations
   - Debug and validate results

4. **Validation with Robotics Toolbox (30 min)**
   - Compare results with library functions
   - Visualize robot motion
   - Understand discrepancies

## Assessment Strategy

### Continuous Assessment (60%)
- Hand-written solutions and derivations (20%)
- Python implementation quality (25%)
- Results validation and analysis (15%)

### Final Exam (40%)
- Theoretical questions (15%)
- Practical problem-solving (25%)

### Grading Criteria
- **Mathematical correctness:** Proper formulation and solution
- **Code quality:** Readability, efficiency, documentation
- **Validation:** Comparison with expected results
- **Understanding:** Ability to explain choices and results

## Study Tips

1. **Review Linear Algebra**
   - Matrix multiplication
   - Inverse and transpose
   - Determinants
   - Eigenvalues/eigenvectors

2. **Practice Python**
   - Complete TP 00 thoroughly
   - Practice NumPy array operations
   - Learn to debug effectively

3. **Visualize Everything**
   - Draw coordinate frames
   - Sketch robot configurations
   - Plot trajectories and velocities

4. **Work Incrementally**
   - Start with simple 2D cases
   - Extend to 3D gradually
   - Test each component separately

5. **Use Resources**
   - Robotics Toolbox documentation
   - NumPy reference guide
   - Course notebooks and examples

## Additional Resources

### Textbooks
- Craig, J.J. "Introduction to Robotics: Mechanics and Control" (3rd Ed.)
- Spong, M.W., et al. "Robot Modeling and Control" (2nd Ed.)
- Siciliano, B., et al. "Robotics: Modelling, Planning and Control"

### Online Resources
- Peter Corke's Robotics Toolbox: https://petercorke.com/toolboxes/robotics-toolbox/
- NumPy Documentation: https://numpy.org/doc/
- Matplotlib Gallery: https://matplotlib.org/stable/gallery/

### Video Lectures
- Angela Sodemann's Robotics Tutorials (YouTube)
- Modern Robotics (Northwestern University)

## Course Schedule Example

| Week | Topic | Hours | Files |
|------|-------|-------|-------|
| 1 | Python Fundamentals | 8 | tp_00 |
| 2-3 | Geometric Transformations | 8 | tp_01 |
| 4-5 | Forward Kinematics | 8 | tp_02 |
| 6-7 | Inverse Kinematics | 8 | tp_03 |
| **Total** | | **32h** | |

## Laboratory Guidelines

### Safety (Even in Simulation!)
- Understand workspace limits
- Check for singularities
- Validate joint limits
- Consider collision avoidance

### Best Practices
1. **Version Control:** Save work frequently
2. **Documentation:** Comment your code clearly
3. **Testing:** Validate with simple cases first
4. **Collaboration:** Discuss with peers, but submit individual work
5. **Questions:** Ask during sessions, don't wait until deadline

## Expected Outcomes

After completing this guided course, students will:

✓ Understand fundamental robot kinematics  
✓ Implement FK/IK algorithms from scratch  
✓ Analyze manipulator velocities and singularities  
✓ Design and execute robot trajectories  
✓ Use professional robotics software tools  
✓ Be prepared for advanced topics (dynamics, control, planning)  
✓ Be ready to tackle the Industry 4.0 SAE project

## Connection to SAE Project

This guided course provides the foundation for the Industry 4.0 SAE (Situation d'Apprentissage et d'Évaluation) project, where students will apply these skills to:

- Design a complete robotic cell
- Implement quality inspection systems
- Integrate vision and motion planning
- Develop industrial applications
- Work with OpenManipulator-X robot

The SAE is a separate 52-hour project-based course that builds directly on the skills developed here.

## Contact and Support

**Instructor:** [Your Name]  
**Office Hours:** [To be announced]  
**Email:** [Your Email]  
**Course Platform:** [LMS/Moodle/etc.]

## Notes

- All notebooks are provided in Jupyter format (.ipynb)
- Solutions are available after submission deadlines
- Late submissions may incur penalties
- Academic integrity is strictly enforced
- Collaboration is encouraged, but copying is not

---

**Last Updated:** November 2025  
**Version:** 1.0  

*This course is continuously improved based on student feedback. Suggestions are welcome!*
