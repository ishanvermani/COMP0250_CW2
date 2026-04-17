# COMP0250_CW2


To get started:

1) Clone or pull the updated surgical vision lab repo
https://github.com/surgical-vision/comp0250_s26_labs

2) navigate to the courseworks directory 

```
cd src/courseworks
```

3) delete `cw2_team_x`

```
rm -rf cw2_team_x
```

4) in `src/courseworks` run

```
git submodule add https://github.com/ishanvermani/COMP0250_CW2.git
```

This will add the cw2_team_11 file, nested in the COMP0250_CW2 folder. This does not impact the build or the code, as the ROS2 package is still installed at the same level as before. 

5) to add coursework 1, run

```
git submodule add https://github.com/oljen/comp0250_cw1_team11.git
```


