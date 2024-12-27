# A6 Servo Initial Setup Procedure

1. Perform Commissioning Procedure accoriding to chapeter 5.3 of A6-RS servo drive manual
2. Perform Inertia Auto-tuning according to Chapter 6.2 of A6-RS servo drive manual
3. Set stiffness level (C00.05), 20 seems to be a good starting point
4. Set C00.02 to 0 (disable simple gear ratio setting), ALF1.0 is displayed because this parameter change requires a power cycle (we will do it after step 7)
5. Ensure that C0A.05 is set to 0 (RS485 register writes shall NOT be stored to EEPROM)
6. Set C0A.06 to 1 (high 16 bits before low 16 bits)
7. Set pulse channel selection (C00.22) to 1 (high speed channel), a power cycle is required after this step
8. Connect controller board
