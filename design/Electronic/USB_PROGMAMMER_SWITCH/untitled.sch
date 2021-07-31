<?xml version="1.0" encoding="utf-8"?>
<!DOCTYPE eagle SYSTEM "eagle.dtd">
<eagle version="9.0.0">
<drawing>
<settings>
<setting alwaysvectorfont="yes"/>
<setting verticaltext="up"/>
</settings>
<grid distance="0.1" unitdist="inch" unit="inch" style="lines" multiple="1" display="no" altdistance="0.01" altunitdist="inch" altunit="inch"/>
<layers>
<layer number="1" name="Top" color="4" fill="1" visible="no" active="no"/>
<layer number="2" name="Route2" color="1" fill="3" visible="no" active="no"/>
<layer number="3" name="Route3" color="4" fill="3" visible="no" active="no"/>
<layer number="4" name="Route4" color="1" fill="4" visible="no" active="no"/>
<layer number="5" name="Route5" color="4" fill="4" visible="no" active="no"/>
<layer number="6" name="Route6" color="1" fill="8" visible="no" active="no"/>
<layer number="7" name="Route7" color="4" fill="8" visible="no" active="no"/>
<layer number="8" name="Route8" color="1" fill="2" visible="no" active="no"/>
<layer number="9" name="Route9" color="4" fill="2" visible="no" active="no"/>
<layer number="10" name="Route10" color="1" fill="7" visible="no" active="no"/>
<layer number="11" name="Route11" color="4" fill="7" visible="no" active="no"/>
<layer number="12" name="Route12" color="1" fill="5" visible="no" active="no"/>
<layer number="13" name="Route13" color="4" fill="5" visible="no" active="no"/>
<layer number="14" name="Route14" color="1" fill="6" visible="no" active="no"/>
<layer number="15" name="Route15" color="4" fill="6" visible="no" active="no"/>
<layer number="16" name="Bottom" color="1" fill="1" visible="no" active="no"/>
<layer number="17" name="Pads" color="2" fill="1" visible="no" active="no"/>
<layer number="18" name="Vias" color="2" fill="1" visible="no" active="no"/>
<layer number="19" name="Unrouted" color="6" fill="1" visible="no" active="no"/>
<layer number="20" name="Dimension" color="15" fill="1" visible="no" active="no"/>
<layer number="21" name="tPlace" color="7" fill="1" visible="no" active="no"/>
<layer number="22" name="bPlace" color="7" fill="1" visible="no" active="no"/>
<layer number="23" name="tOrigins" color="15" fill="1" visible="no" active="no"/>
<layer number="24" name="bOrigins" color="15" fill="1" visible="no" active="no"/>
<layer number="25" name="tNames" color="7" fill="1" visible="no" active="no"/>
<layer number="26" name="bNames" color="7" fill="1" visible="no" active="no"/>
<layer number="27" name="tValues" color="7" fill="1" visible="no" active="no"/>
<layer number="28" name="bValues" color="7" fill="1" visible="no" active="no"/>
<layer number="29" name="tStop" color="7" fill="3" visible="no" active="no"/>
<layer number="30" name="bStop" color="7" fill="6" visible="no" active="no"/>
<layer number="31" name="tCream" color="7" fill="4" visible="no" active="no"/>
<layer number="32" name="bCream" color="7" fill="5" visible="no" active="no"/>
<layer number="33" name="tFinish" color="6" fill="3" visible="no" active="no"/>
<layer number="34" name="bFinish" color="6" fill="6" visible="no" active="no"/>
<layer number="35" name="tGlue" color="7" fill="4" visible="no" active="no"/>
<layer number="36" name="bGlue" color="7" fill="5" visible="no" active="no"/>
<layer number="37" name="tTest" color="7" fill="1" visible="no" active="no"/>
<layer number="38" name="bTest" color="7" fill="1" visible="no" active="no"/>
<layer number="39" name="tKeepout" color="4" fill="11" visible="no" active="no"/>
<layer number="40" name="bKeepout" color="1" fill="11" visible="no" active="no"/>
<layer number="41" name="tRestrict" color="4" fill="10" visible="no" active="no"/>
<layer number="42" name="bRestrict" color="1" fill="10" visible="no" active="no"/>
<layer number="43" name="vRestrict" color="2" fill="10" visible="no" active="no"/>
<layer number="44" name="Drills" color="7" fill="1" visible="no" active="no"/>
<layer number="45" name="Holes" color="7" fill="1" visible="no" active="no"/>
<layer number="46" name="Milling" color="3" fill="1" visible="no" active="no"/>
<layer number="47" name="Measures" color="7" fill="1" visible="no" active="no"/>
<layer number="48" name="Document" color="7" fill="1" visible="no" active="no"/>
<layer number="49" name="Reference" color="7" fill="1" visible="no" active="no"/>
<layer number="51" name="tDocu" color="7" fill="1" visible="no" active="no"/>
<layer number="52" name="bDocu" color="7" fill="1" visible="no" active="no"/>
<layer number="88" name="SimResults" color="9" fill="1" visible="yes" active="yes"/>
<layer number="89" name="SimProbes" color="9" fill="1" visible="yes" active="yes"/>
<layer number="90" name="Modules" color="5" fill="1" visible="yes" active="yes"/>
<layer number="91" name="Nets" color="2" fill="1" visible="yes" active="yes"/>
<layer number="92" name="Busses" color="1" fill="1" visible="yes" active="yes"/>
<layer number="93" name="Pins" color="2" fill="1" visible="no" active="yes"/>
<layer number="94" name="Symbols" color="4" fill="1" visible="yes" active="yes"/>
<layer number="95" name="Names" color="7" fill="1" visible="yes" active="yes"/>
<layer number="96" name="Values" color="7" fill="1" visible="yes" active="yes"/>
<layer number="97" name="Info" color="7" fill="1" visible="yes" active="yes"/>
<layer number="98" name="Guide" color="6" fill="1" visible="yes" active="yes"/>
<layer number="99" name="SpiceOrder" color="5" fill="1" visible="yes" active="yes"/>
<layer number="101" name="Hidden" color="15" fill="1" visible="yes" active="yes"/>
<layer number="102" name="Changes" color="12" fill="1" visible="yes" active="yes"/>
</layers>
<schematic xreflabel="%F%N/%S.%C%R" xrefpart="/%S.%C%R">
<libraries>
<library name="con-usb">
<description>&lt;b&gt;USB Connectors&lt;/b&gt;&lt;p&gt;
&lt;author&gt;Created by Erik Christiansson (erik@alphafish.com)&lt;/author&gt;&lt;p&gt;
Based on the datasheets for the following products.&lt;br&gt;
&lt;table&gt;
&lt;tr&gt;&lt;td&gt;Manufacturer&lt;/td&gt;&lt;td&gt;Part No.&lt;/td&gt;&lt;td&gt;Type&lt;/td&gt;&lt;tr&gt;
&lt;tr&gt;&lt;td&gt;Hsuan Mao&lt;/td&gt;&lt;td&gt;C8317-04AFDXX0&lt;/td&gt;&lt;td&gt;Series A Surface Mounted&lt;/td&gt;&lt;tr&gt;
&lt;tr&gt;&lt;td&gt;Hsuan Mao&lt;/td&gt;&lt;td&gt;C8317-04AFHSW0&lt;/td&gt;&lt;td&gt;Series A Hole Mounted&lt;/td&gt;&lt;tr&gt;
&lt;tr&gt;&lt;td&gt;Hsuan Mao&lt;/td&gt;&lt;td&gt;C3817-04AFVSW0&lt;/td&gt;&lt;td&gt;Series A Hole Mounted Up-Right&lt;/td&gt;&lt;/tr&gt;
&lt;tr&gt;&lt;td&gt;Hsuan Mao&lt;/td&gt;&lt;td&gt;C8317-04BFHSW0&lt;/td&gt;&lt;td&gt;Series B Hole Mounted&lt;/td&gt;&lt;tr&gt;
&lt;tr&gt;&lt;td&gt;Hsuan Mao&lt;/td&gt;&lt;td&gt;C8320-05BFDSB0&lt;/td&gt;&lt;td&gt;Series Mini-B Surface Mounted&lt;/td&gt;&lt;tr&gt;
&lt;tr&gt;&lt;td&gt;Hsuan Mao&lt;/td&gt;&lt;td&gt;C8320-05BFRSB0&lt;/td&gt;&lt;td&gt;Series Mini-B Hole Mounted&lt;/td&gt;&lt;tr&gt;
&lt;tr&gt;&lt;td&gt;Amp&lt;/td&gt;&lt;td&gt;787616-*&lt;/td&gt;&lt;td&gt;Series A Hole Mounted&lt;/td&gt;&lt;tr&gt;
&lt;tr&gt;&lt;td&gt;Amp&lt;/td&gt;&lt;td&gt;787780-1&lt;/td&gt;&lt;td&gt;Series B Hole Mounted&lt;/td&gt;&lt;tr&gt;
&lt;tr&gt;&lt;td&gt;ACON&lt;/td&gt;&lt;td&gt;UAR80-4****0&lt;/td&gt;&lt;td&gt;Series A Hole Mounted Up-Right&lt;/td&gt;&lt;/tr&gt;
&lt;tr&gt;&lt;td&gt;ACON&lt;/td&gt;&lt;td&gt;MNC20-5K5*1*&lt;/td&gt;&lt;td&gt;Series Mini-B Surface Mounted&lt;/td&gt;&lt;tr&gt;
&lt;tr&gt;&lt;td&gt;Molex&lt;/td&gt;&lt;td&gt;67068-****&lt;/td&gt;&lt;td&gt;Series B Hole Mounted&lt;/td&gt;&lt;tr&gt;
&lt;/table&gt;</description>
<packages>
<package name="USB-MB-S">
<description>&lt;b&gt;USB Series Mini-B Surface Mounted&lt;/b&gt;</description>
<wire x1="-5.95" y1="3.85" x2="-5.95" y2="-3.85" width="0.127" layer="22"/>
<wire x1="-5.95" y1="3.85" x2="3.3" y2="3.85" width="0.127" layer="22"/>
<wire x1="3.3" y1="3.85" x2="3.3" y2="-3.85" width="0.127" layer="22"/>
<wire x1="3.3" y1="-3.85" x2="-5.95" y2="-3.85" width="0.127" layer="22"/>
<wire x1="-4.615" y1="1.27" x2="-1.44" y2="0.635" width="0.127" layer="21"/>
<wire x1="-1.44" y1="0.635" x2="-1.44" y2="-0.635" width="0.127" layer="21"/>
<wire x1="-1.44" y1="-0.635" x2="-4.615" y2="-1.27" width="0.127" layer="21"/>
<wire x1="-2.075" y1="-3.175" x2="-4.615" y2="-2.8575" width="0.127" layer="21"/>
<wire x1="-4.615" y1="-2.8575" x2="-4.615" y2="-2.2225" width="0.127" layer="21"/>
<wire x1="-4.615" y1="-2.2225" x2="-2.075" y2="-1.905" width="0.127" layer="21"/>
<wire x1="-2.075" y1="3.175" x2="-4.615" y2="2.8575" width="0.127" layer="21"/>
<wire x1="-4.615" y1="2.8575" x2="-4.615" y2="2.2225" width="0.127" layer="21"/>
<wire x1="-4.615" y1="2.2225" x2="-2.075" y2="1.905" width="0.127" layer="21"/>
<pad name="P$5" x="0" y="2.2" drill="0.9"/>
<pad name="P$6" x="0" y="-2.2" drill="0.9"/>
<smd name="P$1" x="-3" y="-4.5" dx="2.5" dy="2" layer="1"/>
<smd name="P$2" x="-3" y="4.5" dx="2.5" dy="2" layer="1"/>
<smd name="P$3" x="2.5" y="-4.4" dx="2.5" dy="2" layer="1"/>
<smd name="P$4" x="2.5" y="4.5" dx="2.5" dy="2" layer="1"/>
<smd name="D+" x="2.5" y="0" dx="2.5" dy="0.5" layer="1"/>
<smd name="D-" x="2.5" y="0.8" dx="2.5" dy="0.5" layer="1"/>
<smd name="VBUS" x="2.5" y="1.6" dx="2.5" dy="0.5" layer="1"/>
<smd name="ID" x="2.5" y="-0.8" dx="2.5" dy="0.5" layer="1"/>
<smd name="GND" x="2.5" y="-1.6" dx="2.5" dy="0.5" layer="1"/>
<text x="5.2275" y="1.5875" size="1.27" layer="25" rot="R90">&gt;NAME</text>
</package>
<package name="USB-A-H">
<description>&lt;b&gt;USB Series A Hole Mounted&lt;/b&gt;</description>
<wire x1="3.6957" y1="6.5659" x2="-10.287" y2="6.5659" width="0.127" layer="21"/>
<wire x1="3.6957" y1="-6.5659" x2="-10.287" y2="-6.5659" width="0.127" layer="21"/>
<wire x1="-10.287" y1="6.477" x2="-10.287" y2="-6.477" width="0.127" layer="21"/>
<wire x1="3.7084" y1="6.5024" x2="3.7084" y2="-6.5024" width="0.127" layer="21"/>
<wire x1="-2.54" y1="-5.08" x2="-8.89" y2="-4.445" width="0.127" layer="21"/>
<wire x1="-8.89" y1="-4.445" x2="-8.89" y2="-1.27" width="0.127" layer="21"/>
<wire x1="-8.89" y1="-1.27" x2="-2.54" y2="-0.635" width="0.127" layer="21"/>
<wire x1="-2.54" y1="5.08" x2="-8.89" y2="4.445" width="0.127" layer="21"/>
<wire x1="-8.89" y1="4.445" x2="-8.89" y2="1.27" width="0.127" layer="21"/>
<wire x1="-8.89" y1="1.27" x2="-2.54" y2="0.635" width="0.127" layer="21"/>
<pad name="VBUS" x="2.7178" y="3.4925" drill="0.9144" rot="R270"/>
<pad name="D-" x="2.7178" y="1.0033" drill="0.9144" rot="R270"/>
<pad name="D+" x="2.7178" y="-1.0033" drill="0.9144" rot="R270"/>
<pad name="GND" x="2.7178" y="-3.4925" drill="0.9144" rot="R270"/>
<pad name="P$5" x="0" y="-6.5659" drill="2.3114" rot="R270"/>
<pad name="P$6" x="0" y="6.5659" drill="2.3114" rot="R270"/>
<text x="5.715" y="3.81" size="1.27" layer="25" rot="R90">&gt;NAME</text>
</package>
<package name="USB-B-H">
<description>&lt;b&gt;USB Series B Hole Mounted&lt;/b&gt;</description>
<wire x1="-12.486" y1="6.0144" x2="-12.486" y2="-5.9998" width="0.127" layer="21"/>
<wire x1="3.2366" y1="6.0144" x2="3.2366" y2="-5.9998" width="0.127" layer="21"/>
<wire x1="3.2312" y1="6.0198" x2="-12.466" y2="6.0198" width="0.127" layer="21"/>
<wire x1="3.2058" y1="-6.0198" x2="-12.4406" y2="-6.0198" width="0.127" layer="21"/>
<wire x1="-5.08" y1="5.08" x2="-11.43" y2="4.445" width="0.127" layer="21"/>
<wire x1="-11.43" y1="4.445" x2="-11.43" y2="1.27" width="0.127" layer="21"/>
<wire x1="-11.43" y1="1.27" x2="-5.08" y2="0.635" width="0.127" layer="21"/>
<wire x1="-5.08" y1="-5.08" x2="-11.43" y2="-4.445" width="0.127" layer="21"/>
<wire x1="-11.43" y1="-4.445" x2="-11.43" y2="-1.27" width="0.127" layer="21"/>
<wire x1="-11.43" y1="-1.27" x2="-5.08" y2="-0.635" width="0.127" layer="21"/>
<pad name="VBUS" x="1.9812" y="-1.25" drill="0.9144" rot="R90"/>
<pad name="D-" x="1.9812" y="1.25" drill="0.9144" rot="R90"/>
<pad name="D+" x="0" y="1.25" drill="0.9144" rot="R270"/>
<pad name="GND" x="0" y="-1.25" drill="0.9144" rot="R270"/>
<pad name="P$1" x="-2.7178" y="-6.0198" drill="2.286"/>
<pad name="P$2" x="-2.7178" y="6.0198" drill="2.286"/>
<text x="5.06" y="3.175" size="1.27" layer="25" rot="R90">&gt;NAME</text>
</package>
<package name="USB-A-S">
<description>&lt;b&gt;USB Series A Surface Mounted&lt;/b&gt;</description>
<wire x1="3.6957" y1="6.5659" x2="-10.287" y2="6.5659" width="0.127" layer="21"/>
<wire x1="3.6957" y1="-6.5659" x2="-10.287" y2="-6.5659" width="0.127" layer="21"/>
<wire x1="-10.287" y1="6.477" x2="-10.287" y2="-6.477" width="0.127" layer="21"/>
<wire x1="3.7084" y1="6.5024" x2="3.7084" y2="-6.5024" width="0.127" layer="21"/>
<wire x1="-2.54" y1="-5.08" x2="-8.89" y2="-4.445" width="0.127" layer="21"/>
<wire x1="-8.89" y1="-4.445" x2="-8.89" y2="-1.27" width="0.127" layer="21"/>
<wire x1="-8.89" y1="-1.27" x2="-2.54" y2="-0.635" width="0.127" layer="21"/>
<wire x1="-2.54" y1="5.08" x2="-8.89" y2="4.445" width="0.127" layer="21"/>
<wire x1="-8.89" y1="4.445" x2="-8.89" y2="1.27" width="0.127" layer="21"/>
<wire x1="-8.89" y1="1.27" x2="-2.54" y2="0.635" width="0.127" layer="21"/>
<pad name="P$5" x="0" y="-6.5659" drill="2.3114" rot="R270"/>
<pad name="P$6" x="0" y="6.5659" drill="2.3114" rot="R270"/>
<smd name="D-" x="3.45" y="1" dx="3" dy="0.9" layer="1"/>
<smd name="VBUS" x="3.45" y="3" dx="3" dy="0.9" layer="1"/>
<smd name="D+" x="3.45" y="-1" dx="3" dy="0.9" layer="1"/>
<smd name="GND" x="3.45" y="-3" dx="3" dy="0.9" layer="1"/>
<text x="5.715" y="3.81" size="1.27" layer="25" rot="R90">&gt;NAME</text>
</package>
<package name="USB-MB-H">
<description>&lt;b&gt;USB Series Mini-B Hole Mounted&lt;/b&gt;</description>
<wire x1="-3.75" y1="3.9" x2="-3.75" y2="-3.9" width="0.127" layer="22"/>
<wire x1="5.25" y1="3.9" x2="5.25" y2="-3.9" width="0.127" layer="22"/>
<wire x1="-3.75" y1="3.9" x2="5.25" y2="3.9" width="0.127" layer="22"/>
<wire x1="-3.75" y1="-3.9" x2="5.25" y2="-3.9" width="0.127" layer="22"/>
<wire x1="0.75" y1="3.5" x2="-3.25" y2="3" width="0.127" layer="22"/>
<wire x1="-3.25" y1="3" x2="-3.25" y2="2" width="0.127" layer="22"/>
<wire x1="-3.25" y1="2" x2="0.75" y2="1.5" width="0.127" layer="22"/>
<wire x1="1.25" y1="-3.5" x2="-3.25" y2="-3" width="0.127" layer="22"/>
<wire x1="-3.25" y1="-3" x2="-3.25" y2="-2" width="0.127" layer="22"/>
<wire x1="-3.25" y1="-2" x2="1.25" y2="-1.5" width="0.127" layer="22"/>
<wire x1="-3.25" y1="1.25" x2="1.75" y2="0.75" width="0.127" layer="22"/>
<wire x1="1.75" y1="0.75" x2="1.75" y2="-0.75" width="0.127" layer="22"/>
<wire x1="1.75" y1="-0.75" x2="-3.25" y2="-1.25" width="0.127" layer="22"/>
<pad name="VBUS" x="5.1" y="1.6" drill="0.8"/>
<pad name="D+" x="5.1" y="0" drill="0.8"/>
<pad name="GND" x="5.1" y="-1.6" drill="0.8"/>
<pad name="D-" x="3.9" y="0.8" drill="0.8"/>
<pad name="ID" x="3.9" y="-0.8" drill="0.8"/>
<pad name="P$6" x="0" y="-3.65" drill="1.9"/>
<pad name="P$7" x="0" y="3.65" drill="1.9"/>
<text x="7.25" y="1.5" size="1.27" layer="25" rot="R90">&gt;NAME</text>
<rectangle x1="3.25" y1="3" x2="5.75" y2="4.4" layer="43"/>
<rectangle x1="3.25" y1="-4.4" x2="5.75" y2="-3" layer="43"/>
<rectangle x1="-3.75" y1="-3.1" x2="-1.425" y2="3.1" layer="43"/>
<rectangle x1="-1.425" y1="-2.325" x2="-0.65" y2="2.325" layer="43"/>
</package>
<package name="USB-A-HU">
<description>&lt;b&gt;USB Series A Hole Mounted Up-Right&lt;/b&gt;</description>
<wire x1="-11.6" y1="2.6" x2="-11.6" y2="-2.6" width="0.127" layer="21"/>
<wire x1="7.7" y1="2.6" x2="7.7" y2="-2.6" width="0.127" layer="21"/>
<wire x1="-11.6" y1="2.6" x2="7.7" y2="2.6" width="0.127" layer="21"/>
<wire x1="-11.6" y1="-2.6" x2="7.7" y2="-2.6" width="0.127" layer="21"/>
<pad name="GND" x="0.73" y="0" drill="0.9"/>
<pad name="D+" x="2.73" y="0" drill="0.9"/>
<pad name="D-" x="4.73" y="0" drill="0.9"/>
<pad name="VBUS" x="6.73" y="0" drill="0.9"/>
<pad name="P$5" x="0" y="2.72" drill="1.5"/>
<pad name="P$6" x="0" y="-2.72" drill="1.5"/>
<pad name="P$7" x="7" y="2.72" drill="1.5"/>
<pad name="P$8" x="7" y="-2.72" drill="1.5"/>
<text x="10" y="1" size="1.27" layer="25" rot="R90">&gt;NAME</text>
</package>
<package name="USB-B-SMT">
<description>USB Series B Surface Mounted</description>
<wire x1="-9" y1="6" x2="-9" y2="-6" width="0.127" layer="21"/>
<wire x1="-9" y1="-6" x2="-1" y2="-6" width="0.127" layer="21"/>
<wire x1="-1" y1="-6" x2="2.4" y2="-6" width="0.127" layer="21"/>
<wire x1="2.4" y1="-6" x2="7" y2="-6" width="0.127" layer="21"/>
<wire x1="7" y1="-6" x2="7" y2="6" width="0.127" layer="21"/>
<wire x1="7" y1="6" x2="2.4" y2="6" width="0.127" layer="21"/>
<wire x1="2.4" y1="6" x2="-1" y2="6" width="0.127" layer="21"/>
<wire x1="-1" y1="6" x2="-9" y2="6" width="0.127" layer="21"/>
<wire x1="2.4" y1="6" x2="2.4" y2="7.3" width="0.127" layer="21"/>
<wire x1="2.4" y1="7.3" x2="2.2" y2="7.5" width="0.127" layer="21"/>
<wire x1="2.2" y1="7.5" x2="1.9" y2="7.5" width="0.127" layer="21"/>
<wire x1="1.9" y1="7.5" x2="1.4" y2="7" width="0.127" layer="21"/>
<wire x1="-1" y1="6" x2="-1" y2="7.3" width="0.127" layer="21"/>
<wire x1="-0.4" y1="7.4" x2="-0.3" y2="7.5" width="0.127" layer="22"/>
<wire x1="-0.8" y1="7.5" x2="-0.5" y2="7.5" width="0.127" layer="21"/>
<wire x1="-0.5" y1="7.5" x2="0" y2="7" width="0.127" layer="21"/>
<wire x1="0" y1="7" x2="1.4" y2="7" width="0.127" layer="21"/>
<wire x1="-1" y1="-6" x2="-1" y2="-7.3" width="0.127" layer="21"/>
<wire x1="-1" y1="-7.3" x2="-0.8" y2="-7.5" width="0.127" layer="21"/>
<wire x1="-0.8" y1="-7.5" x2="-0.5" y2="-7.5" width="0.127" layer="21"/>
<wire x1="-0.5" y1="-7.5" x2="0" y2="-7" width="0.127" layer="21"/>
<wire x1="3" y1="-7.4" x2="2.9" y2="-7.5" width="0.127" layer="22"/>
<wire x1="1.9" y1="-7.5" x2="1.4" y2="-7" width="0.127" layer="21"/>
<wire x1="1.4" y1="-7" x2="0" y2="-7" width="0.127" layer="21"/>
<wire x1="-1" y1="7.3" x2="-0.8" y2="7.5" width="0.127" layer="21"/>
<wire x1="2.2" y1="-7.5" x2="1.9" y2="-7.5" width="0.127" layer="21"/>
<wire x1="2.2" y1="-7.5" x2="2.4" y2="-7.3" width="0.127" layer="21"/>
<wire x1="2.4" y1="-6" x2="2.4" y2="-7.3" width="0.127" layer="21"/>
<wire x1="6" y1="0.5" x2="5" y2="0.5" width="0.0504" layer="21"/>
<wire x1="5" y1="0.5" x2="5" y2="-0.5" width="0.127" layer="21"/>
<wire x1="5" y1="-0.5" x2="6" y2="-0.5" width="0.0504" layer="21"/>
<smd name="5" x="0.58" y="6.8" dx="6.04" dy="3.4" layer="1"/>
<smd name="6" x="0.58" y="-6.8" dx="6.04" dy="3.4" layer="1"/>
<smd name="D+" x="7" y="1.875" dx="3" dy="0.7" layer="1"/>
<smd name="D-" x="7" y="0.625" dx="3" dy="0.7" layer="1"/>
<smd name="GND" x="7" y="-0.625" dx="3" dy="0.7" layer="1"/>
<smd name="VUSB" x="7" y="-1.875" dx="3" dy="0.7" layer="1"/>
<text x="9.525" y="6.35" size="1.27" layer="25" rot="R270">&gt;NAME</text>
<hole x="0" y="2.25" drill="1.4"/>
<hole x="0" y="-2.25" drill="1.4"/>
</package>
</packages>
<symbols>
<symbol name="USB">
<wire x1="5.08" y1="8.89" x2="0" y2="8.89" width="0.254" layer="94"/>
<wire x1="0" y1="8.89" x2="0" y2="-1.27" width="0.254" layer="94"/>
<wire x1="0" y1="-1.27" x2="5.08" y2="-1.27" width="0.254" layer="94"/>
<text x="3.81" y="0" size="2.54" layer="94" rot="R90">USB</text>
<pin name="D+" x="-2.54" y="7.62" visible="pad" length="short"/>
<pin name="D-" x="-2.54" y="5.08" visible="pad" length="short"/>
<pin name="VBUS" x="-2.54" y="2.54" visible="pad" length="short"/>
<pin name="GND" x="-2.54" y="0" visible="pad" length="short"/>
</symbol>
</symbols>
<devicesets>
<deviceset name="USB" prefix="X">
<description>&lt;b&gt;USB Connectors&lt;/b&gt;&lt;p&gt;</description>
<gates>
<gate name="G$1" symbol="USB" x="0" y="0"/>
</gates>
<devices>
<device name="-MB-S" package="USB-MB-S">
<connects>
<connect gate="G$1" pin="D+" pad="D+"/>
<connect gate="G$1" pin="D-" pad="D-"/>
<connect gate="G$1" pin="GND" pad="GND"/>
<connect gate="G$1" pin="VBUS" pad="VBUS"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="-A-H" package="USB-A-H">
<connects>
<connect gate="G$1" pin="D+" pad="D+"/>
<connect gate="G$1" pin="D-" pad="D-"/>
<connect gate="G$1" pin="GND" pad="GND"/>
<connect gate="G$1" pin="VBUS" pad="VBUS"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="-B-H" package="USB-B-H">
<connects>
<connect gate="G$1" pin="D+" pad="D+"/>
<connect gate="G$1" pin="D-" pad="D-"/>
<connect gate="G$1" pin="GND" pad="GND"/>
<connect gate="G$1" pin="VBUS" pad="VBUS"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="-A-S" package="USB-A-S">
<connects>
<connect gate="G$1" pin="D+" pad="D+"/>
<connect gate="G$1" pin="D-" pad="D-"/>
<connect gate="G$1" pin="GND" pad="GND"/>
<connect gate="G$1" pin="VBUS" pad="VBUS"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="-MB-H" package="USB-MB-H">
<connects>
<connect gate="G$1" pin="D+" pad="D+"/>
<connect gate="G$1" pin="D-" pad="D-"/>
<connect gate="G$1" pin="GND" pad="GND"/>
<connect gate="G$1" pin="VBUS" pad="VBUS"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="-A-HU" package="USB-A-HU">
<connects>
<connect gate="G$1" pin="D+" pad="D+"/>
<connect gate="G$1" pin="D-" pad="D-"/>
<connect gate="G$1" pin="GND" pad="GND"/>
<connect gate="G$1" pin="VBUS" pad="VBUS"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="-B-S" package="USB-B-SMT">
<connects>
<connect gate="G$1" pin="D+" pad="D+"/>
<connect gate="G$1" pin="D-" pad="D-"/>
<connect gate="G$1" pin="GND" pad="GND"/>
<connect gate="G$1" pin="VBUS" pad="VUSB"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
</devices>
</deviceset>
</devicesets>
</library>
<library name="relay" urn="urn:adsk.eagle:library:339">
<description>&lt;b&gt;Relays&lt;/b&gt;&lt;p&gt;
&lt;ul&gt;
&lt;li&gt;Eichhoff
&lt;li&gt;Finder
&lt;li&gt;Fujitsu
&lt;li&gt;HAMLIN
&lt;li&gt;OMRON
&lt;li&gt;Matsushita
&lt;li&gt;NAiS
&lt;li&gt;Siemens
&lt;li&gt;Schrack
&lt;/ul&gt;
&lt;author&gt;Created by librarian@cadsoft.de&lt;/author&gt;</description>
<packages>
<package name="FRS1B" urn="urn:adsk.eagle:footprint:23997/1" library_version="1">
<description>&lt;b&gt;RELAY&lt;/b&gt;&lt;p&gt;
1 x switch, same as V23101-D6-A201, Siemens</description>
<wire x1="10.16" y1="2.413" x2="10.16" y2="0" width="0.1524" layer="21"/>
<wire x1="10.16" y1="0" x2="-1.651" y2="0" width="0.1524" layer="21"/>
<wire x1="10.16" y1="0" x2="10.16" y2="-2.413" width="0.1524" layer="21"/>
<wire x1="-2.794" y1="-1.143" x2="-1.651" y2="0" width="0.254" layer="21"/>
<wire x1="-2.54" y1="-0.635" x2="-2.54" y2="-2.032" width="0.1524" layer="21"/>
<wire x1="-2.54" y1="0.762" x2="-2.54" y2="2.032" width="0.1524" layer="21"/>
<wire x1="4.191" y1="-0.762" x2="4.191" y2="-1.524" width="0.1524" layer="21"/>
<wire x1="4.191" y1="-1.524" x2="2.794" y2="-1.524" width="0.1524" layer="21"/>
<wire x1="2.794" y1="-1.524" x2="2.794" y2="-2.794" width="0.1524" layer="21"/>
<wire x1="2.794" y1="-2.794" x2="3.81" y2="-2.794" width="0.1524" layer="21"/>
<wire x1="5.588" y1="-2.794" x2="5.588" y2="-1.524" width="0.1524" layer="21"/>
<wire x1="5.588" y1="-1.524" x2="4.572" y2="-1.524" width="0.1524" layer="21"/>
<wire x1="4.191" y1="-2.794" x2="4.191" y2="-3.81" width="0.1524" layer="21"/>
<wire x1="4.191" y1="-2.794" x2="5.588" y2="-2.794" width="0.1524" layer="21"/>
<wire x1="4.191" y1="-3.81" x2="1.143" y2="-3.81" width="0.1524" layer="21"/>
<wire x1="4.572" y1="-1.524" x2="3.81" y2="-2.794" width="0.1524" layer="21"/>
<wire x1="4.572" y1="-1.524" x2="4.191" y2="-1.524" width="0.1524" layer="21"/>
<wire x1="3.81" y1="-2.794" x2="4.191" y2="-2.794" width="0.1524" layer="21"/>
<wire x1="-4.064" y1="-5.08" x2="-4.064" y2="5.08" width="0.1524" layer="21"/>
<wire x1="-4.064" y1="-5.08" x2="-3.556" y2="-5.588" width="0.1524" layer="21" curve="90"/>
<wire x1="11.43" y1="5.588" x2="11.938" y2="5.08" width="0.1524" layer="21" curve="-90"/>
<wire x1="11.43" y1="-5.588" x2="11.938" y2="-5.08" width="0.1524" layer="21" curve="90"/>
<wire x1="-4.064" y1="5.08" x2="-3.556" y2="5.588" width="0.1524" layer="21" curve="-90"/>
<wire x1="-3.556" y1="5.588" x2="11.43" y2="5.588" width="0.1524" layer="21"/>
<wire x1="-3.556" y1="-5.588" x2="11.43" y2="-5.588" width="0.1524" layer="21"/>
<wire x1="11.938" y1="-5.08" x2="11.938" y2="5.08" width="0.1524" layer="21"/>
<wire x1="0" y1="2.032" x2="0" y2="-0.762" width="0.1524" layer="21"/>
<wire x1="0" y1="-0.762" x2="4.191" y2="-0.762" width="0.1524" layer="21"/>
<circle x="10.16" y="0" radius="0.127" width="0.1524" layer="21"/>
<pad name="O" x="-2.54" y="-3.81" drill="0.9144" shape="long" rot="R90"/>
<pad name="S" x="-2.54" y="3.81" drill="0.9144" shape="long" rot="R90"/>
<pad name="1" x="0" y="3.81" drill="0.9144" shape="long" rot="R90"/>
<pad name="2" x="0" y="-3.81" drill="0.9144" shape="long" rot="R90"/>
<pad name="P" x="10.16" y="3.81" drill="0.9144" shape="long" rot="R90"/>
<pad name="P@" x="10.16" y="-3.81" drill="0.9144" shape="long" rot="R90"/>
<text x="-4.445" y="-5.08" size="1.27" layer="25" ratio="10" rot="R90">&gt;NAME</text>
<text x="2.032" y="3.302" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
</package>
</packages>
<packages3d>
<package3d name="FRS1B" urn="urn:adsk.eagle:package:24333/1" type="box" library_version="1">
<description>RELAY
1 x switch, same as V23101-D6-A201, Siemens</description>
<packageinstances>
<packageinstance name="FRS1B"/>
</packageinstances>
</package3d>
</packages3d>
<symbols>
<symbol name="K" urn="urn:adsk.eagle:symbol:23941/1" library_version="1">
<wire x1="-3.81" y1="-1.905" x2="-1.905" y2="-1.905" width="0.254" layer="94"/>
<wire x1="3.81" y1="-1.905" x2="3.81" y2="1.905" width="0.254" layer="94"/>
<wire x1="3.81" y1="1.905" x2="1.905" y2="1.905" width="0.254" layer="94"/>
<wire x1="-3.81" y1="1.905" x2="-3.81" y2="-1.905" width="0.254" layer="94"/>
<wire x1="0" y1="-2.54" x2="0" y2="-1.905" width="0.1524" layer="94"/>
<wire x1="0" y1="-1.905" x2="3.81" y2="-1.905" width="0.254" layer="94"/>
<wire x1="0" y1="2.54" x2="0" y2="1.905" width="0.1524" layer="94"/>
<wire x1="0" y1="1.905" x2="-3.81" y2="1.905" width="0.254" layer="94"/>
<wire x1="-1.905" y1="-1.905" x2="1.905" y2="1.905" width="0.1524" layer="94"/>
<wire x1="-1.905" y1="-1.905" x2="0" y2="-1.905" width="0.254" layer="94"/>
<wire x1="1.905" y1="1.905" x2="0" y2="1.905" width="0.254" layer="94"/>
<text x="1.27" y="2.921" size="1.778" layer="96">&gt;VALUE</text>
<text x="1.27" y="5.08" size="1.778" layer="95">&gt;PART</text>
<pin name="2" x="0" y="-5.08" visible="pad" length="short" direction="pas" rot="R90"/>
<pin name="1" x="0" y="5.08" visible="pad" length="short" direction="pas" rot="R270"/>
</symbol>
<symbol name="U1" urn="urn:adsk.eagle:symbol:23981/1" library_version="1">
<wire x1="3.175" y1="5.08" x2="1.905" y2="5.08" width="0.254" layer="94"/>
<wire x1="-3.175" y1="5.08" x2="-1.905" y2="5.08" width="0.254" layer="94"/>
<wire x1="0" y1="1.27" x2="2.54" y2="5.715" width="0.254" layer="94"/>
<wire x1="-2.54" y1="0" x2="0" y2="0" width="0.1524" layer="94"/>
<wire x1="0" y1="0" x2="0" y2="1.27" width="0.254" layer="94"/>
<circle x="0" y="0" radius="0.127" width="0.4064" layer="94"/>
<circle x="0" y="1.27" radius="0.127" width="0.4064" layer="94"/>
<text x="2.54" y="0" size="1.778" layer="95">&gt;PART</text>
<pin name="O" x="5.08" y="5.08" visible="pad" length="short" direction="pas" rot="R180"/>
<pin name="S" x="-5.08" y="5.08" visible="pad" length="short" direction="pas"/>
<pin name="P" x="0" y="-2.54" visible="pad" length="short" direction="pas" swaplevel="1" rot="R90"/>
<pin name="P@1" x="-2.54" y="-2.54" visible="pad" length="short" direction="pas" swaplevel="1" rot="R90"/>
</symbol>
</symbols>
<devicesets>
<deviceset name="FRS1B" urn="urn:adsk.eagle:component:24607/1" prefix="K" library_version="1">
<description>&lt;b&gt;RELAY&lt;/b&gt;&lt;p&gt;
1 x switch, same as V23101-D6-A201, Siemens</description>
<gates>
<gate name="1" symbol="K" x="0" y="0" addlevel="must"/>
<gate name="2" symbol="U1" x="15.24" y="0" addlevel="always"/>
</gates>
<devices>
<device name="" package="FRS1B">
<connects>
<connect gate="1" pin="1" pad="1"/>
<connect gate="1" pin="2" pad="2"/>
<connect gate="2" pin="O" pad="O"/>
<connect gate="2" pin="P" pad="P"/>
<connect gate="2" pin="P@1" pad="P@"/>
<connect gate="2" pin="S" pad="S"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:24333/1"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="MF" value="" constant="no"/>
<attribute name="MPN" value="" constant="no"/>
<attribute name="OC_FARNELL" value="unknown" constant="no"/>
<attribute name="OC_NEWARK" value="unknown" constant="no"/>
</technology>
</technologies>
</device>
</devices>
</deviceset>
</devicesets>
</library>
<library name="supply1" urn="urn:adsk.eagle:library:371">
<description>&lt;b&gt;Supply Symbols&lt;/b&gt;&lt;p&gt;
 GND, VCC, 0V, +5V, -5V, etc.&lt;p&gt;
 Please keep in mind, that these devices are necessary for the
 automatic wiring of the supply signals.&lt;p&gt;
 The pin name defined in the symbol is identical to the net which is to be wired automatically.&lt;p&gt;
 In this library the device names are the same as the pin names of the symbols, therefore the correct signal names appear next to the supply symbols in the schematic.&lt;p&gt;
 &lt;author&gt;Created by librarian@cadsoft.de&lt;/author&gt;</description>
<packages>
</packages>
<symbols>
<symbol name="GND" urn="urn:adsk.eagle:symbol:26925/1" library_version="1">
<wire x1="-1.905" y1="0" x2="1.905" y2="0" width="0.254" layer="94"/>
<text x="-2.54" y="-2.54" size="1.778" layer="96">&gt;VALUE</text>
<pin name="GND" x="0" y="2.54" visible="off" length="short" direction="sup" rot="R270"/>
</symbol>
</symbols>
<devicesets>
<deviceset name="GND" urn="urn:adsk.eagle:component:26954/1" prefix="GND" library_version="1">
<description>&lt;b&gt;SUPPLY SYMBOL&lt;/b&gt;</description>
<gates>
<gate name="1" symbol="GND" x="0" y="0"/>
</gates>
<devices>
<device name="">
<technologies>
<technology name=""/>
</technologies>
</device>
</devices>
</deviceset>
</devicesets>
</library>
<library name="diode" urn="urn:adsk.eagle:library:210">
<description>&lt;b&gt;Diodes&lt;/b&gt;&lt;p&gt;
Based on the following sources:
&lt;ul&gt;
&lt;li&gt;Motorola : www.onsemi.com
&lt;li&gt;Fairchild : www.fairchildsemi.com
&lt;li&gt;Philips : www.semiconductors.com
&lt;li&gt;Vishay : www.vishay.de
&lt;/ul&gt;
&lt;author&gt;Created by librarian@cadsoft.de&lt;/author&gt;</description>
<packages>
<package name="DO41-10" urn="urn:adsk.eagle:footprint:43094/1" library_version="2">
<description>&lt;B&gt;DIODE&lt;/B&gt;&lt;p&gt;
diameter 2.54 mm, horizontal, grid 10.16 mm</description>
<wire x1="2.032" y1="-1.27" x2="-2.032" y2="-1.27" width="0.1524" layer="21"/>
<wire x1="2.032" y1="-1.27" x2="2.032" y2="1.27" width="0.1524" layer="21"/>
<wire x1="-2.032" y1="1.27" x2="2.032" y2="1.27" width="0.1524" layer="21"/>
<wire x1="-2.032" y1="1.27" x2="-2.032" y2="-1.27" width="0.1524" layer="21"/>
<wire x1="5.08" y1="0" x2="4.064" y2="0" width="0.762" layer="51"/>
<wire x1="-5.08" y1="0" x2="-4.064" y2="0" width="0.762" layer="51"/>
<wire x1="-0.635" y1="0" x2="0" y2="0" width="0.1524" layer="21"/>
<wire x1="1.016" y1="0.635" x2="1.016" y2="-0.635" width="0.1524" layer="21"/>
<wire x1="1.016" y1="-0.635" x2="0" y2="0" width="0.1524" layer="21"/>
<wire x1="0" y1="0" x2="1.524" y2="0" width="0.1524" layer="21"/>
<wire x1="0" y1="0" x2="1.016" y2="0.635" width="0.1524" layer="21"/>
<wire x1="0" y1="0.635" x2="0" y2="0" width="0.1524" layer="21"/>
<wire x1="0" y1="0" x2="0" y2="-0.635" width="0.1524" layer="21"/>
<pad name="A" x="5.08" y="0" drill="1.1176"/>
<pad name="C" x="-5.08" y="0" drill="1.1176"/>
<text x="-2.032" y="1.651" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-2.032" y="-2.794" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
<rectangle x1="-1.651" y1="-1.27" x2="-1.143" y2="1.27" layer="21"/>
<rectangle x1="2.032" y1="-0.381" x2="3.937" y2="0.381" layer="21"/>
<rectangle x1="-3.937" y1="-0.381" x2="-2.032" y2="0.381" layer="21"/>
</package>
</packages>
<packages3d>
<package3d name="DO41-10" urn="urn:adsk.eagle:package:43336/1" type="box" library_version="2">
<description>DIODE
diameter 2.54 mm, horizontal, grid 10.16 mm</description>
<packageinstances>
<packageinstance name="DO41-10"/>
</packageinstances>
</package3d>
</packages3d>
<symbols>
<symbol name="D" urn="urn:adsk.eagle:symbol:43091/1" library_version="2">
<wire x1="-1.27" y1="-1.27" x2="1.27" y2="0" width="0.254" layer="94"/>
<wire x1="1.27" y1="0" x2="-1.27" y2="1.27" width="0.254" layer="94"/>
<wire x1="1.27" y1="1.27" x2="1.27" y2="0" width="0.254" layer="94"/>
<wire x1="-1.27" y1="1.27" x2="-1.27" y2="-1.27" width="0.254" layer="94"/>
<wire x1="1.27" y1="0" x2="1.27" y2="-1.27" width="0.254" layer="94"/>
<text x="2.54" y="0.4826" size="1.778" layer="95">&gt;NAME</text>
<text x="2.54" y="-2.3114" size="1.778" layer="96">&gt;VALUE</text>
<text x="-2.54" y="0" size="0.4064" layer="99" align="center">SpiceOrder 1</text>
<text x="2.54" y="0" size="0.4064" layer="99" align="center">SpiceOrder 2</text>
<pin name="A" x="-2.54" y="0" visible="off" length="short" direction="pas"/>
<pin name="C" x="2.54" y="0" visible="off" length="short" direction="pas" rot="R180"/>
</symbol>
</symbols>
<devicesets>
<deviceset name="1N4004" urn="urn:adsk.eagle:component:43462/1" prefix="D" library_version="2">
<description>&lt;B&gt;DIODE&lt;/B&gt;&lt;p&gt;
general purpose rectifier, 1 A</description>
<gates>
<gate name="1" symbol="D" x="0" y="0"/>
</gates>
<devices>
<device name="" package="DO41-10">
<connects>
<connect gate="1" pin="A" pad="A"/>
<connect gate="1" pin="C" pad="C"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:43336/1"/>
</package3dinstances>
<technologies>
<technology name=""/>
</technologies>
</device>
</devices>
</deviceset>
</devicesets>
</library>
</libraries>
<attributes>
</attributes>
<variantdefs>
</variantdefs>
<classes>
<class number="0" name="default" width="0" drill="0">
</class>
</classes>
<parts>
<part name="X1" library="con-usb" deviceset="USB" device="-B-H"/>
<part name="K1" library="relay" library_urn="urn:adsk.eagle:library:339" deviceset="FRS1B" device="" package3d_urn="urn:adsk.eagle:package:24333/1"/>
<part name="X2" library="con-usb" deviceset="USB" device="-A-H"/>
<part name="X3" library="con-usb" deviceset="USB" device="-A-H"/>
<part name="K2" library="relay" library_urn="urn:adsk.eagle:library:339" deviceset="FRS1B" device="" package3d_urn="urn:adsk.eagle:package:24333/1"/>
<part name="GND1" library="supply1" library_urn="urn:adsk.eagle:library:371" deviceset="GND" device=""/>
<part name="GND2" library="supply1" library_urn="urn:adsk.eagle:library:371" deviceset="GND" device=""/>
<part name="GND3" library="supply1" library_urn="urn:adsk.eagle:library:371" deviceset="GND" device=""/>
<part name="K3" library="relay" library_urn="urn:adsk.eagle:library:339" deviceset="FRS1B" device="" package3d_urn="urn:adsk.eagle:package:24333/1"/>
<part name="D1" library="diode" library_urn="urn:adsk.eagle:library:210" deviceset="1N4004" device="" package3d_urn="urn:adsk.eagle:package:43336/1"/>
</parts>
<sheets>
<sheet>
<plain>
</plain>
<instances>
<instance part="X1" gate="G$1" x="15.24" y="55.88" rot="R180"/>
<instance part="K1" gate="1" x="38.1" y="93.98" rot="R180">
<attribute name="VALUE" x="36.83" y="91.059" size="1.778" layer="96" rot="R180"/>
<attribute name="PART" x="36.83" y="88.9" size="1.778" layer="95" rot="R180"/>
</instance>
<instance part="K1" gate="2" x="50.8" y="55.88" rot="R90">
<attribute name="PART" x="50.8" y="58.42" size="1.778" layer="95" rot="R90"/>
</instance>
<instance part="X2" gate="G$1" x="15.24" y="78.74" rot="R180"/>
<instance part="X3" gate="G$1" x="86.36" y="50.8"/>
<instance part="K2" gate="1" x="25.4" y="93.98"/>
<instance part="K2" gate="2" x="50.8" y="43.18" rot="R90"/>
<instance part="GND1" gate="1" x="86.36" y="43.18"/>
<instance part="GND2" gate="1" x="17.78" y="60.96" rot="R180"/>
<instance part="GND3" gate="1" x="20.32" y="81.28" rot="R180"/>
<instance part="K3" gate="1" x="48.26" y="93.98"/>
<instance part="K3" gate="2" x="50.8" y="71.12" rot="R90"/>
<instance part="D1" gate="1" x="27.94" y="86.36"/>
</instances>
<busses>
</busses>
<nets>
<net name="N$4" class="0">
<segment>
<wire x1="53.34" y1="43.18" x2="73.66" y2="43.18" width="0.1524" layer="91"/>
<wire x1="73.66" y1="43.18" x2="73.66" y2="58.42" width="0.1524" layer="91"/>
<pinref part="X3" gate="G$1" pin="D+"/>
<wire x1="73.66" y1="58.42" x2="83.82" y2="58.42" width="0.1524" layer="91"/>
<pinref part="K2" gate="2" pin="P"/>
<pinref part="K2" gate="2" pin="P@1"/>
<wire x1="53.34" y1="43.18" x2="53.34" y2="40.64" width="0.1524" layer="91"/>
<junction x="53.34" y="43.18"/>
</segment>
</net>
<net name="GND" class="0">
<segment>
<pinref part="X2" gate="G$1" pin="GND"/>
<pinref part="GND3" gate="1" pin="GND"/>
<wire x1="17.78" y1="78.74" x2="20.32" y2="78.74" width="0.1524" layer="91"/>
<pinref part="K2" gate="1" pin="1"/>
<wire x1="25.4" y1="99.06" x2="17.78" y2="99.06" width="0.1524" layer="91"/>
<wire x1="17.78" y1="99.06" x2="17.78" y2="86.36" width="0.1524" layer="91"/>
<junction x="17.78" y="78.74"/>
<pinref part="D1" gate="1" pin="A"/>
<wire x1="17.78" y1="86.36" x2="17.78" y2="78.74" width="0.1524" layer="91"/>
<wire x1="25.4" y1="86.36" x2="17.78" y2="86.36" width="0.1524" layer="91"/>
<junction x="17.78" y="86.36"/>
</segment>
<segment>
<pinref part="X3" gate="G$1" pin="GND"/>
<pinref part="GND1" gate="1" pin="GND"/>
<wire x1="83.82" y1="50.8" x2="83.82" y2="45.72" width="0.1524" layer="91"/>
<wire x1="83.82" y1="45.72" x2="86.36" y2="45.72" width="0.1524" layer="91"/>
</segment>
<segment>
<pinref part="GND2" gate="1" pin="GND"/>
<pinref part="X1" gate="G$1" pin="GND"/>
<wire x1="17.78" y1="58.42" x2="17.78" y2="55.88" width="0.1524" layer="91"/>
</segment>
</net>
<net name="N$1" class="0">
<segment>
<pinref part="K3" gate="2" pin="P"/>
<pinref part="K3" gate="2" pin="P@1"/>
<wire x1="53.34" y1="71.12" x2="53.34" y2="68.58" width="0.1524" layer="91"/>
<wire x1="53.34" y1="68.58" x2="63.5" y2="68.58" width="0.1524" layer="91"/>
<wire x1="63.5" y1="68.58" x2="63.5" y2="53.34" width="0.1524" layer="91"/>
<pinref part="X3" gate="G$1" pin="VBUS"/>
<wire x1="63.5" y1="53.34" x2="83.82" y2="53.34" width="0.1524" layer="91"/>
<junction x="53.34" y="68.58"/>
</segment>
</net>
<net name="N$2" class="0">
<segment>
<pinref part="K1" gate="2" pin="P"/>
<pinref part="X3" gate="G$1" pin="D-"/>
<wire x1="53.34" y1="55.88" x2="83.82" y2="55.88" width="0.1524" layer="91"/>
<pinref part="K1" gate="2" pin="P@1"/>
<wire x1="53.34" y1="55.88" x2="53.34" y2="53.34" width="0.1524" layer="91"/>
<junction x="53.34" y="55.88"/>
</segment>
</net>
<net name="N$3" class="0">
<segment>
<pinref part="X1" gate="G$1" pin="D-"/>
<wire x1="17.78" y1="50.8" x2="45.72" y2="50.8" width="0.1524" layer="91"/>
<pinref part="K1" gate="2" pin="S"/>
</segment>
</net>
<net name="N$5" class="0">
<segment>
<pinref part="X1" gate="G$1" pin="D+"/>
<pinref part="K2" gate="2" pin="S"/>
<wire x1="17.78" y1="48.26" x2="17.78" y2="38.1" width="0.1524" layer="91"/>
<wire x1="17.78" y1="38.1" x2="45.72" y2="38.1" width="0.1524" layer="91"/>
</segment>
</net>
<net name="N$6" class="0">
<segment>
<pinref part="X2" gate="G$1" pin="D+"/>
<wire x1="17.78" y1="71.12" x2="38.1" y2="71.12" width="0.1524" layer="91"/>
<pinref part="K2" gate="2" pin="O"/>
<wire x1="38.1" y1="71.12" x2="38.1" y2="48.26" width="0.1524" layer="91"/>
<wire x1="38.1" y1="48.26" x2="45.72" y2="48.26" width="0.1524" layer="91"/>
</segment>
</net>
<net name="N$7" class="0">
<segment>
<pinref part="X2" gate="G$1" pin="D-"/>
<wire x1="17.78" y1="73.66" x2="40.64" y2="73.66" width="0.1524" layer="91"/>
<wire x1="40.64" y1="73.66" x2="40.64" y2="60.96" width="0.1524" layer="91"/>
<pinref part="K1" gate="2" pin="O"/>
<wire x1="40.64" y1="60.96" x2="45.72" y2="60.96" width="0.1524" layer="91"/>
</segment>
</net>
<net name="N$8" class="0">
<segment>
<pinref part="X1" gate="G$1" pin="VBUS"/>
<wire x1="17.78" y1="53.34" x2="33.02" y2="53.34" width="0.1524" layer="91"/>
<wire x1="33.02" y1="53.34" x2="33.02" y2="66.04" width="0.1524" layer="91"/>
<pinref part="K3" gate="2" pin="S"/>
<wire x1="33.02" y1="66.04" x2="45.72" y2="66.04" width="0.1524" layer="91"/>
<wire x1="33.02" y1="66.04" x2="33.02" y2="83.82" width="0.1524" layer="91"/>
<wire x1="33.02" y1="83.82" x2="48.26" y2="83.82" width="0.1524" layer="91"/>
<pinref part="K3" gate="1" pin="2"/>
<wire x1="48.26" y1="83.82" x2="48.26" y2="88.9" width="0.1524" layer="91"/>
<pinref part="D1" gate="1" pin="C"/>
<wire x1="30.48" y1="86.36" x2="30.48" y2="83.82" width="0.1524" layer="91"/>
<wire x1="30.48" y1="83.82" x2="33.02" y2="83.82" width="0.1524" layer="91"/>
<junction x="33.02" y="83.82"/>
</segment>
</net>
<net name="N$9" class="0">
<segment>
<pinref part="X2" gate="G$1" pin="VBUS"/>
<pinref part="K3" gate="2" pin="O"/>
<wire x1="17.78" y1="76.2" x2="45.72" y2="76.2" width="0.1524" layer="91"/>
</segment>
</net>
<net name="N$10" class="0">
<segment>
<pinref part="K2" gate="1" pin="2"/>
<pinref part="K1" gate="1" pin="1"/>
<wire x1="25.4" y1="88.9" x2="38.1" y2="88.9" width="0.1524" layer="91"/>
</segment>
</net>
<net name="N$11" class="0">
<segment>
<pinref part="K1" gate="1" pin="2"/>
<pinref part="K3" gate="1" pin="1"/>
<wire x1="38.1" y1="99.06" x2="48.26" y2="99.06" width="0.1524" layer="91"/>
</segment>
</net>
</nets>
</sheet>
</sheets>
</schematic>
</drawing>
<compatibility>
<note version="8.2" severity="warning">
Since Version 8.2, EAGLE supports online libraries. The ids
of those online libraries will not be understood (or retained)
with this version.
</note>
<note version="8.3" severity="warning">
Since Version 8.3, EAGLE supports URNs for individual library
assets (packages, symbols, and devices). The URNs of those assets
will not be understood (or retained) with this version.
</note>
<note version="8.3" severity="warning">
Since Version 8.3, EAGLE supports the association of 3D packages
with devices in libraries, schematics, and board files. Those 3D
packages will not be understood (or retained) with this version.
</note>
</compatibility>
</eagle>
