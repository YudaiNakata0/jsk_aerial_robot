<?xml version="1.0"?>
<launch>
  <!-- parameters -->
  <arg name="lowerb_hue_1" default="0"/>
  <arg name="lowerb_hue_2" default="170"/>
  <arg name="upperb_hue_1" default="10"/>
  <arg name="upperb_hue_2" default="180"/>
  <arg name="lowerb_luminance" default="50"/>
  <arg name="upperb_luminance" default="255"/>
  <arg name="lowerb_saturation" default="50"/>
  <arg name="upperb_saturation" default="255"/>
  <arg name="threshold" default="20"/>
  <arg name="minLineLength" default="40"/>
  <arg name="maxLineGap" default="20"/>

  <!-- script -->
  <node pkg="camera" type="camera_red_detection.py" name="camera_processing_node" output="screen">
    <param name="lh1" value="$(arg lowerb_hue_1)"/>
    <param name="lh2" value="$(arg lowerb_hue_2)"/>
    <param name="uh1" value="$(arg upperb_hue_1)"/>
    <param name="uh2" value="$(arg upperb_hue_2)"/>
    <param name="ll" value="$(arg lowerb_luminance)"/>
    <param name="ul" value="$(arg upperb_luminance)"/>
    <param name="ls" value="$(arg lowerb_saturation)"/>
    <param name="us" value="$(arg upperb_saturation)"/>
    <param name="threshold" value="$(arg threshold)"/>
    <param name="minLineLength" value="$(arg minLineLength)"/>
    <param name="maxLineGap" value="$(arg maxLineGap)"/>
  </node>
</launch>
