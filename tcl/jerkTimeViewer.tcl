#!/bin/sh
# the next line restarts using wish \
exec elwish "$0" "$@"
#  *------------------------------------------------------------*
#  * 
#  *                     --  Jerk Times Viewer   --
#  * 
#  *  DESCRIPTION: Compute optimal time motion and adjust time slowing velocity
#  *  CREATION DATE : MON AUG 20 09:00:00 2007 
#  *  MODIFICATION : Plot vector when slider are moved
#  *  AUTHOR: DANIEL SIDOBRE (bltplot) & XAVIER BROQUERE (JerkTimeViewer)
#  *  
#  *------------------------------------------------------------*
lappend auto_path /usr/local/openrobots/tclTools
lappend auto_path /home/xbroquer/openrobots/src/tclTools

package require BLT
namespace import blt::*
package require editval
package require bltplot

package require jerk

#load /home/xbroquer/openrobots/src/softMotion-libs/installs/lib/jerk.so

#Declaration de la fenetre de l'interface
set w .image1

#Initialisation des variables
set jmax 0.6
set amax 0.2
set vmax 0.1
set a0 0
set v0 0
set x0 0
set af 0
set vf 0
set xf 0
set chemin "Type de chemin"
set timp 1

set xf 0.028868
set v0 0.05
set vf 0.05

# Decalartion des structures Tcl
set limits [jerkParams]
set ic [jerkConditions]
set fc [jerkConditions]
set jerkdata [jerkData]
set jerkTimesAdjusted [jerkTimesAdjusted]
set jerkTimesAdjustedVc1Vc2 [jerkTimesAdjustedVc1Vc2]

set icfond [jerkConditions] 
set fcfond [jerkConditions]
set jerkdatafond [jerkData]

# Destruction des widgets existants
if { [winfo exist $w.Limits] == 1 } {
    eval destroy [winfo children .]
}
if { [winfo exist .graph] == 1 } {
    destroy .graph
}
if { [winfo exist .graph1] == 1 } {
    destroy .graph1
}

#  *------------------------------------------------------------*
#  * 
#  *                     --  Procedures  --
#  * 
#  *------------------------------------------------------------*
set jerkTimesVectorExit 0

proc graphique_jerk_vectors { nom args } {

    if {$::jerkTimesVectorExit == 0} {
        set ::jerkTimesVectorExit 1
       graphique_fermer $nom
       frame .$nom
       label .$nom.nb_fenetre -text -0
       label .$nom.num_fenetre -text -1
       label .$nom.vecteurs -text ""
       
       set vecteurs ""
       namespace eval $nom "vector create t_vectors"
       lappend vecteurs t_vectors
       namespace eval $nom "vector create J_vectors"
       lappend vecteurs J_vectors
       namespace eval $nom "vector create A_vectors"
       lappend vecteurs A_vectors
       namespace eval $nom "vector create V_vectors"
       lappend vecteurs V_vectors
       namespace eval $nom "vector create X_vectors"
       lappend vecteurs X_vectors
       
       namespace eval $nom "t_vectors set {$::st} "
       namespace eval $nom "J_vectors set {$::vectJ} "
       namespace eval $nom "A_vectors set {$::vectA} "
       namespace eval $nom "V_vectors set {$::vectV} "
       namespace eval $nom "X_vectors set {$::vectX} "
       namespace eval $nom {set legende "t temps J jerk" }
       .$nom.vecteurs configure -text $vecteurs
       graphique_creer [set vecteurs] $nom
       graphique_echelle .graph.0 {t_vectors J_vectors A_vectors V_vectors X_vectors}
   } else {
	     set vecteurs ""
       namespace eval $nom "vector create t_vectors"
       lappend vecteurs t_vectors
       namespace eval $nom "vector create J_vectors"
       lappend vecteurs J_vectors
       namespace eval $nom "vector create A_vectors"
       lappend vecteurs A_vectors
       namespace eval $nom "vector create V_vectors"
       lappend vecteurs V_vectors
       namespace eval $nom "vector create X_vectors"
       lappend vecteurs X_vectors
       
       namespace eval $nom "t_vectors set {$::st} "
       namespace eval $nom "J_vectors set {$::vectJ} "
       namespace eval $nom "A_vectors set {$::vectA} "
       namespace eval $nom "V_vectors set {$::vectV} "
       namespace eval $nom "X_vectors set {$::vectX} "
	}
}

set jerkTimesExit 0

proc graphique_jerk_times {nom args} {

    if {$::jerkTimesExit == 0} {
	    set ::jerkTimesExit 1
	    graphique_fermer $nom
	    frame .$nom
	    label .$nom.nb_fenetre -text -0
	    label .$nom.num_fenetre -text -1
	    label .$nom.vecteurs -text ""
	    set vecteurs2 ""	    
	    namespace eval $nom "vector create V_times"
	    lappend vecteurs2 V_times
	    namespace eval $nom "vector create A_times"
	    lappend vecteurs2 A_times
	    namespace eval $nom "vector create Vfond"
	    namespace eval $nom "vector create Afond"
	    namespace eval $nom "V_times set {$::vectV} "
	    namespace eval $nom "A_times set {$::vectA} "
	    namespace eval $nom "Vfond set {$::vectVfond} "
	    namespace eval $nom "Afond set {$::vectAfond} "

	    .$nom.vecteurs configure -text $vecteurs2
	    graphique_creer [set vecteurs2] $nom

	    namespace eval $nom ".graph1.0.g element create line -xdata Vfond -ydata Afond"

	    button .graph1.0.1.zoomspecial -text ZoomSpecial -command { .graph1.0.g axis configure A -max [expr $::amax + 0.01] -min [expr -$::amax - 0.01]; .graph1.0.g axis configure y -max [expr $::amax + 0.01] -min [expr -$::amax - 0.01]; .graph1.0.g axis configure x -max [expr $::vmax + 0.01] -min [expr -$::vmax - 0.01]}
	    pack .graph1.0.1.zoomspecial -side left 
	} else {
	    set vecteurs2 "" 
	    namespace eval $nom "vector create V_times"
	    lappend vecteurs2 V_times
	    namespace eval $nom "vector create A_times"
	    lappend vecteurs2 A_times
	    namespace eval $nom "vector create Vfond"
	    namespace eval $nom "vector create Afond"
	    namespace eval $nom "V_times set {$::vectV} "
	    namespace eval $nom "A_times set {$::vectA} "
	    namespace eval $nom "Vfond set {$::vectVfond} "
	    namespace eval $nom "Afond set {$::vectAfond} "   
	    namespace eval $nom ".graph1.0.g element configure line -xdata Vfond -ydata Afond"
	    .graph1.0.g element configure line -symbol none
	   
	    .$nom.vecteurs configure -text $vecteurs2

	    .graph1.0.1.zoomspecial configure  -command { .graph1.0.g axis configure A -max [expr $::amax + 0.01] -min [expr -$::amax - 0.01]; .graph1.0.g axis configure y -max [expr $::amax + 0.01] -min [expr -$::amax - 0.01]; .graph1.0.g axis configure x -max [expr $::vmax + 0.01] -min [expr -$::vmax - 0.01]}
	}
	
	.$nom.vecteurs configure -text $vecteurs2
	
#graphique_echelle .graph1.0 { A}
	    
	.graph1.0.g axis configure x -max [expr $::vmax + 0.01] -min [expr -$::vmax - 0.01]
	.graph1.0.g axis configure y -max [expr $::amax + 0.01] -min [expr -$::amax - 0.01]
	.graph1.0.g axis configure A_times -max [expr $::amax + 0.01] -min [expr -$::amax - 0.01]
	    
#	.graph1.0.g marker create bitmap -name bg \
#	-coords "[expr -$::vmax-0.001] -$::amax [expr $::vmax + 0.002] [expr $::amax + 0.001]"  \
#	-bitmap  @./Jerk/backgr.xbm \
#	-under yes \
#	-background white
	    
	.graph1.0.g marker create text  -name p1  -coords "[expr $::v0 + 0.01] [expr $::a0 + 0]" -text P1 -foreground green  -under yes -background white
	.graph1.0.g marker create text  -name p11 -text X -coords "[expr $::v0 + 0] [expr $::a0 + 0]" -foreground green -font 4 -under yes -background white
	.graph1.0.g marker create text  -name p2  -coords "[expr $::vf - 0.01] [expr $::af + 0]" -text P2 -foreground red -under yes -background white
	.graph1.0.g marker create text  -name p21 -text X -coords "$::Vf2 $::Af2" -foreground red -font 4 -under yes -background white
}



proc calcul_jerk { args } {

    set ::x0 0
    $::limits configure -Jmax $::jmax -Amax $::amax -Vmax $::vmax
    $::ic configure -A $::a0 -V $::v0 -X $::x0
    $::fc configure -A $::af -V $::vf -X $::xf

    jerkTimes $::limits $::ic $::fc $::jerkdata

    set ::t1 [format %.5f [jerkData_t1_get $::jerkdata]]
    set ::t2 [format %.5f [jerkData_t2_get $::jerkdata]]
    set ::t3 [format %.5f [jerkData_t3_get $::jerkdata]]
    set ::t4 [format %.5f [jerkData_t4_get $::jerkdata]]
    set ::t5 [format %.5f [jerkData_t5_get $::jerkdata]]
    set ::t6 [format %.5f [jerkData_t6_get $::jerkdata]]
    set ::t7 [format %.5f [jerkData_t7_get $::jerkdata]]
    set ::dir [jerkData_dir_get $::jerkdata]
    set ::dc [format %.5f [jerkData_dc_get $::jerkdata]]
    set ::zone [jerkData_zone_get $::jerkdata]

    if {$::dir == 1} {
	set ::chemin "Chemin type 1"
    } else {
	set ::chemin "Chemin type 2"
	set ::dir -1
    }
    set ::totalTime [expr $::t1 + $::t2 + $::t3 + $::t4 + $::t5 + $::t6 + $::t7]

 #    if { [winfo exist $::w.conditions.adjustTime.timeImp] == 1 } {
#      $::w.conditions.adjustTime.timeImp configure -min $::totalTime
#     }
  

    #set SamplingTime and 1/SamplingTime
    set std 1000
    set Ts 0.001

    set st1 [expr round($::t1 * $std) ] 
    set st2 [expr round($::t2 * $std) ] 
    set st3 [expr round($::t3 * $std) ]
    set st4 [expr round($::t4 * $std) ]
    set st5 [expr round($::t5 * $std) ]
    set st6 [expr round($::t6 * $std) ]
    set st7 [expr round($::t7 * $std) ]
    set NOE [expr $st1 + $st2 + $st3 + $st4 + $st5 + $st6 + $st7 -1]

    # Building Vectors
    set Ts2 [expr $Ts * $Ts]
    set Ts3 [expr $Ts2 * $Ts]   
    set k 0 
    set ::st 0
    set ::vectJ 0
    set ::vectA $::a0
    set ::vectV $::v0
    set ::vectX 0

    for {set i 0} {$i < $NOE-1} {incr i 1} {
	lappend ::st $i
    }

    # Tjpa Interval 
    set ::Ai  $::a0
    set ::Vi  $::v0
    set ::Xi  $::x0
    if {  $st1 > 1 } {
	for {set i 0} {$i <= $st1} {incr i 1} {
	    incr k 1
	    set k1 [expr $Ts * $i]
	    set k2 [expr 0.5 * $Ts2 * $i *  $i] 
	    set k3 [expr   $i * $i * $i* $Ts3 / 6]
	    lappend ::vectJ $::jmax
	    lappend ::vectA [expr $::a0 + $::dir * $k1 * $::jmax]
	    lappend ::vectV [expr $::v0 + $k1 * $::a0 + $::dir * $k2 * $::jmax]
	    lappend ::vectX [expr $::x0 + $k1 * $::v0 + $k2 * $::a0 + $::dir * $k3 * $::jmax]
	}
	set ::Ai [lindex $::vectA [expr $k]]
	set ::Vi [lindex $::vectV [expr $k]]
	set ::Xi [lindex $::vectX [expr $k]]
    }
    if {  $st2 > 1 } {
	# Taca Interval  
	for {set i 0} {$i <= $st2} {incr i 1} {
	    incr k 1
	    set k1 [expr $Ts * $i]
	    set k2 [expr 0.5 * $Ts2 * $i *  $i] 
	    lappend ::vectJ 0
	    lappend ::vectA $::Ai
	    lappend ::vectV [expr $::Vi + $k1 * $::Ai]
	    lappend ::vectX [expr $::Xi + $k1 * $::Vi + $k2 * $::Ai]	    
	}   
	set ::Ai [lindex $::vectA [expr $k]]
	set ::Vi [lindex $::vectV [expr $k]]
	set ::Xi [lindex $::vectX [expr $k]]
    }
    if {  $st3 > 1 } {
	# Tjna Interval  
	for {set i 1} {$i <= $st3} {incr i 1} {
	    incr k 1
	    set k1 [expr $Ts * $i]
	    set k2 [expr 0.5 * $Ts2 * $i *  $i] 
	    set k3 [expr   $i * $i * $i* $Ts3 / 6]
	    lappend ::vectJ [expr -$::jmax]
	    lappend ::vectA [expr $::Ai - $::dir * $k1 * $::jmax] 
	    lappend ::vectV [expr $::Vi + $k1 * $::Ai - $::dir * $k2 * $::jmax]
	    lappend ::vectX [expr $::Xi + $k1 * $::Vi + $k2 * $::Ai - $::dir * $k3 * $::jmax]
	}   
	set ::Ai [lindex $::vectA [expr $k]]
	set ::Vi [lindex $::vectV [expr $k]]
	set ::Xi [lindex $::vectX [expr $k]]
    }
    if {  $st4 > 1 } {
	# Tjvc Interval  
	for {set i 1} {$i <= $st4} {incr i 1} {
	    incr k 1
	    set k1 [expr $Ts * $i]
	    lappend ::vectJ 0
	    lappend ::vectA 0
	    lappend ::vectV [expr $::Vi]
	    lappend ::vectX [expr $::Xi + $k1 * $::Vi] 
	}   
	set ::Ai [lindex $::vectA [expr $k]]
	set ::Vi [lindex $::vectV [expr $k]]
	set ::Xi [lindex $::vectX [expr $k]]
    }
    if {  $st5 > 1 } {
	# Tjnb Interval  
	for {set i 1} {$i <= $st5} {incr i 1} {
	    incr k 1
	    set k1 [expr $Ts * $i]
	    set k2 [expr 0.5 * $Ts2 * $i *  $i] 
	    set k3 [expr   $i * $i * $i* $Ts3 / 6]
	    lappend ::vectJ [expr -$::jmax]
	    lappend ::vectA [expr $::Ai - $::dir * $k1 * $::jmax] 
	    lappend ::vectV [expr $::Vi + $k1 * $::Ai - $::dir * $k2 * $::jmax]
	    lappend ::vectX [expr $::Xi + $k1 * $::Vi + $k2 * $::Ai - $::dir * $k3 * $::jmax]   
	}   
	set ::Ai [lindex $::vectA [expr $k]]
	set ::Vi [lindex $::vectV [expr $k]]
	set ::Xi [lindex $::vectX [expr $k]]
    }
    if {  $st6 > 1 } {
	# Tacb Interval  
	for {set i 0} {$i <= $st6} {incr i 1} {
	    incr k 1
	    set k1 [expr $Ts * $i]
	    set k2 [expr 0.5 * $Ts2 * $i *  $i]   
	    lappend ::vectJ 0
	    lappend ::vectA $::Ai
	    lappend ::vectV [expr $::Vi + $k1 * $::Ai]
	    lappend ::vectX [expr $::Xi + $k1 * $::Vi + $k2 * $::Ai]  
	}   
	set ::Ai [lindex $::vectA [expr $k]]
	set ::Vi [lindex $::vectV [expr $k]]
	set ::Xi [lindex $::vectX [expr $k]]
    }
    if {  $st7 > 1 } {
	# Tjpb Interval  
	for {set i 0} {$i <= $st7} {incr i 1} {
	    incr k 1
	    set k1 [expr $Ts * $i]
	    set k2 [expr 0.5 * $Ts2 * $i *  $i] 
	    set k3 [expr   $i * $i * $i* $Ts3 / 6]
	    lappend ::vectJ $::jmax
	    lappend ::vectA [expr $::Ai + $::dir * $k1 * $::jmax]
	    lappend ::vectV [expr $::Vi + $k1 * $::Ai + $::dir * $k2 * $::jmax]
	    lappend ::vectX [expr $::Xi + $k1 * $::Vi + $k2 * $::Ai + $::dir * $k3 * $::jmax]  
	}
    }
    set ::Af2 [lindex $::vectA [expr $k]]
    set ::Vf2 [lindex $::vectV [expr $k]]
    set ::Xf2 [lindex $::vectX [expr $k]]
}

proc calcul_jerk_adjusted { args } {
    
    set ::x0 0
    $::limits configure -Jmax $::jmax -Amax $::amax -Vmax $::vmax
    $::jerkTimesAdjusted configure -timp $::timp
    $::ic configure -A $::a0 -V $::v0 -X $::x0
    $::fc configure -A $::af -V $::vf -X $::xf
    
    
    
#      jerkTimes $::limits $::ic $::fc $::jerkdata
    
#      set t1tmp [format %.5f [jerkData_t1_get $::jerkdata]]
#      set t2tmp [format %.5f [jerkData_t2_get $::jerkdata]]
#      set t3tmp [format %.5f [jerkData_t3_get $::jerkdata]]
#      set t4tmp [format %.5f [jerkData_t4_get $::jerkdata]]
#      set t5tmp [format %.5f [jerkData_t5_get $::jerkdata]]
#      set t6tmp [format %.5f [jerkData_t6_get $::jerkdata]]
#      set t7tmp [format %.5f [jerkData_t7_get $::jerkdata]]
#      set ::totalTimetmp [expr $t1tmp + $t2tmp + $t3tmp + $t4tmp + $t5tmp + $t6tmp + $t7tmp]
puts $::totalTime

     $::w.conditions.adjustTime.timeImp configure -min $::totalTime
    
    if { $::timp <= $::totalTime } {
	    puts "Timp < TimeMin : do nothing"
	    
	} else {
	   puts "it will not work check me !!" 
	 #   adjustTimeSlowingVc $::limits $::ic $::fc $::jerkTimesAdjusted

    set ::t1Adj [format %.5f [jerkTimesAdjusted_t1_get $::jerkTimesAdjusted]]
    set ::t2Adj [format %.5f [jerkTimesAdjusted_t2_get $::jerkTimesAdjusted]]
    set ::t3Adj [format %.5f [jerkTimesAdjusted_t3_get $::jerkTimesAdjusted]]
    set ::t4Adj [format %.5f [jerkTimesAdjusted_t4_get $::jerkTimesAdjusted]]
    set ::t5Adj [format %.5f [jerkTimesAdjusted_t5_get $::jerkTimesAdjusted]]
    set ::t6Adj [format %.5f [jerkTimesAdjusted_t6_get $::jerkTimesAdjusted]]
    set ::t7Adj [format %.5f [jerkTimesAdjusted_t7_get $::jerkTimesAdjusted]]

    set ::dir_a [jerkTimesAdjusted_dir_a_get $::jerkTimesAdjusted]
    set ::dir_b [jerkTimesAdjusted_dir_b_get $::jerkTimesAdjusted]


    set ::totalTimeAdj [expr $::t1Adj + $::t2Adj + $::t3Adj + $::t4Adj + $::t5Adj + $::t6Adj + $::t7Adj]

    #set SamplingTime and 1/SamplingTime
    set std 1000
    set Ts 0.001

    set st1 [expr round($::t1Adj * $std) ] 
    set st2 [expr round($::t2Adj * $std) ] 
    set st3 [expr round($::t3Adj * $std) ]
    set st4 [expr round($::t4Adj * $std) ]
    set st5 [expr round($::t5Adj * $std) ]
    set st6 [expr round($::t6Adj * $std) ]
    set st7 [expr round($::t7Adj * $std) ]
    set NOE [expr $st1 + $st2 + $st3 + $st4 + $st5 + $st6 + $st7 -1]

    # Building Vectors
    set Ts2 [expr $Ts * $Ts]
    set Ts3 [expr $Ts2 * $Ts]   
    set k 0 
    set ::st 0
    set ::vectJ 0
    set ::vectA $::a0
    set ::vectV $::v0
    set ::vectX 0

    for {set i 0} {$i < $NOE-1} {incr i 1} {
	lappend ::st $i
    }

    # Tjpa Interval 
    set ::Ai  $::a0
    set ::Vi  $::v0
    set ::Xi  $::x0
    if {  $st1 > 1 } {
	for {set i 0} {$i <= $st1} {incr i 1} {
	    incr k 1
	    set k1 [expr $Ts * $i]
	    set k2 [expr 0.5 * $Ts2 * $i *  $i] 
	    set k3 [expr   $i * $i * $i* $Ts3 / 6]
	    lappend ::vectJ [expr $::dir_a*$::jmax]
	    lappend ::vectA [expr $::a0 + $::dir_a * $k1 * $::jmax]
	    lappend ::vectV [expr $::v0 + $k1 * $::a0 + $::dir_a * $k2 * $::jmax]
	    lappend ::vectX [expr $::x0 + $k1 * $::v0 + $k2 * $::a0 + $::dir_a * $k3 * $::jmax]
	}
	set ::Ai [lindex $::vectA [expr $k]]
	set ::Vi [lindex $::vectV [expr $k]]
	set ::Xi [lindex $::vectX [expr $k]]
    }
    if {  $st2 > 1 } {
	# Taca Interval  
	for {set i 0} {$i <= $st2} {incr i 1} {
	    incr k 1
	    set k1 [expr $Ts * $i]
	    set k2 [expr 0.5 * $Ts2 * $i *  $i] 
	    lappend ::vectJ 0
	    lappend ::vectA $::Ai
	    lappend ::vectV [expr $::Vi + $k1 * $::Ai]
	    lappend ::vectX [expr $::Xi + $k1 * $::Vi + $k2 * $::Ai]	    
	}   
	set ::Ai [lindex $::vectA [expr $k]]
	set ::Vi [lindex $::vectV [expr $k]]
	set ::Xi [lindex $::vectX [expr $k]]
    }
    if {  $st3 > 1 } {
	# Tjna Interval  
	for {set i 1} {$i <= $st3} {incr i 1} {
	    incr k 1
	    set k1 [expr $Ts * $i]
	    set k2 [expr 0.5 * $Ts2 * $i *  $i] 
	    set k3 [expr   $i * $i * $i* $Ts3 / 6]
	    lappend ::vectJ [expr -$::dir_a*$::jmax]
	    lappend ::vectA [expr $::Ai - $::dir_a * $k1 * $::jmax] 
	    lappend ::vectV [expr $::Vi + $k1 * $::Ai - $::dir_a * $k2 * $::jmax]
	    lappend ::vectX [expr $::Xi + $k1 * $::Vi + $k2 * $::Ai - $::dir_a * $k3 * $::jmax]
	}   
	set ::Ai [lindex $::vectA [expr $k]]
	set ::Vi [lindex $::vectV [expr $k]]
	set ::Xi [lindex $::vectX [expr $k]]
    }
    if {  $st4 > 1 } {
	# Tvc Interval  
	for {set i 1} {$i <= $st4} {incr i 1} {
	    incr k 1
	    set k1 [expr $Ts * $i]
	    lappend ::vectJ 0
	    lappend ::vectA 0
	    lappend ::vectV [expr $::Vi]
	    lappend ::vectX [expr $::Xi + $k1 * $::Vi] 
	}   
	set ::Ai [lindex $::vectA [expr $k]]
	set ::Vi [lindex $::vectV [expr $k]]
	set ::Xi [lindex $::vectX [expr $k]]
    }
    if {  $st5 > 1 } {
	# Tjnb Interval  
	for {set i 1} {$i <= $st5} {incr i 1} {
	    incr k 1
	    set k1 [expr $Ts * $i]
	    set k2 [expr 0.5 * $Ts2 * $i *  $i] 
	    set k3 [expr   $i * $i * $i* $Ts3 / 6]
	    lappend ::vectJ [expr $::dir_b *$::jmax]
	    lappend ::vectA [expr $::Ai + $::dir_b * $k1 * $::jmax] 
	    lappend ::vectV [expr $::Vi + $k1 * $::Ai + $::dir_b * $k2 * $::jmax]
	    lappend ::vectX [expr $::Xi + $k1 * $::Vi + $k2 * $::Ai + $::dir_b * $k3 * $::jmax]   
	}   
	set ::Ai [lindex $::vectA [expr $k]]
	set ::Vi [lindex $::vectV [expr $k]]
	set ::Xi [lindex $::vectX [expr $k]]
    }
    if {  $st6 > 1 } {
	# Tacb Interval  
	for {set i 0} {$i <= $st6} {incr i 1} {
	    incr k 1
	    set k1 [expr $Ts * $i]
	    set k2 [expr 0.5 * $Ts2 * $i *  $i]   
	    lappend ::vectJ 0
	    lappend ::vectA $::Ai
	    lappend ::vectV [expr $::Vi + $k1 * $::Ai]
	    lappend ::vectX [expr $::Xi + $k1 * $::Vi + $k2 * $::Ai]  
	}   
	set ::Ai [lindex $::vectA [expr $k]]
	set ::Vi [lindex $::vectV [expr $k]]
	set ::Xi [lindex $::vectX [expr $k]]
    }
    if {  $st7 > 1 } {
	# Tjpb Interval  
	for {set i 0} {$i <= $st7} {incr i 1} {
	    incr k 1
	    set k1 [expr $Ts * $i]
	    set k2 [expr 0.5 * $Ts2 * $i *  $i] 
	    set k3 [expr   $i * $i * $i* $Ts3 / 6]
	    lappend ::vectJ [expr - $::dir_b * $::jmax]
	    lappend ::vectA [expr $::Ai - $::dir_b * $k1 * $::jmax]
	    lappend ::vectV [expr $::Vi + $k1 * $::Ai - $::dir_b * $k2 * $::jmax]
	    lappend ::vectX [expr $::Xi + $k1 * $::Vi + $k2 * $::Ai - $::dir_b * $k3 * $::jmax]  
	}
    }
    set ::Af2 [lindex $::vectA [expr $k]]
    set ::Vf2 [lindex $::vectV [expr $k]]
    set ::Xf2 [lindex $::vectX [expr $k]]
}
}

proc calcul_jerk_adjustedVc1Vc2 { args } {
    
    set ::x0 0
    $::limits configure -Jmax $::jmax -Amax $::amax -Vmax $::vmax
    $::jerkTimesAdjustedVc1Vc2 configure -timp $::timp
    $::ic configure -A $::a0 -V $::v0 -X $::x0
    $::fc configure -A $::af -V $::vf -X $::xf
    
#      jerkTimes $::limits $::ic $::fc $::jerkdata 
#      set t1tmp [format %.5f [jerkData_t1_get $::jerkdata]]
#      set t2tmp [format %.5f [jerkData_t2_get $::jerkdata]]
#      set t3tmp [format %.5f [jerkData_t3_get $::jerkdata]]
#      set t4tmp [format %.5f [jerkData_t4_get $::jerkdata]]
#      set t5tmp [format %.5f [jerkData_t5_get $::jerkdata]]
#      set t6tmp [format %.5f [jerkData_t6_get $::jerkdata]]
#      set t7tmp [format %.5f [jerkData_t7_get $::jerkdata]]
#      set ::totalTimetmp [expr $t1tmp + $t2tmp + $t3tmp + $t4tmp + $t5tmp + $t6tmp + $t7tmp]
    puts $::totalTime

     $::w.conditions.adjustTime.timeImp configure -min $::totalTime
    
    if { $::timp <= $::totalTime } {
	    puts "Timp < TimeMin : do nothing"
	    
	} else {
	    
	    adjustTimeSlowingVc1Vc2 $::limits $::ic $::fc $::jerkTimesAdjustedVc1Vc2

    set ::tjpaAdj [format %.5f [jerkTimesAdjustedVc1Vc2_Tjpa_get $::jerkTimesAdjustedVc1Vc2]]
    set ::tacaAdj [format %.5f [jerkTimesAdjustedVc1Vc2_Taca_get $::jerkTimesAdjustedVc1Vc2]]
    set ::tjnaAdj [format %.5f [jerkTimesAdjustedVc1Vc2_Tjna_get $::jerkTimesAdjustedVc1Vc2]]
    set ::tvc1Adj [format %.5f [jerkTimesAdjustedVc1Vc2_Tvc1_get $::jerkTimesAdjustedVc1Vc2]]
    set ::tjnbAdj [format %.5f [jerkTimesAdjustedVc1Vc2_Tjnb_get $::jerkTimesAdjustedVc1Vc2]]
    set ::tacbAdj [format %.5f [jerkTimesAdjustedVc1Vc2_Tacb_get $::jerkTimesAdjustedVc1Vc2]]
    set ::tjpbAdj [format %.5f [jerkTimesAdjustedVc1Vc2_Tjpb_get $::jerkTimesAdjustedVc1Vc2]]
    set ::tvc2Adj [format %.5f [jerkTimesAdjustedVc1Vc2_Tvc2_get $::jerkTimesAdjustedVc1Vc2]]
    set ::tjpcAdj [format %.5f [jerkTimesAdjustedVc1Vc2_Tjpc_get $::jerkTimesAdjustedVc1Vc2]]
    set ::taccAdj [format %.5f [jerkTimesAdjustedVc1Vc2_Tacc_get $::jerkTimesAdjustedVc1Vc2]]
    set ::tjncAdj [format %.5f [jerkTimesAdjustedVc1Vc2_Tjnc_get $::jerkTimesAdjustedVc1Vc2]]

    set ::totalTimeAdj [expr $::tjpaAdj + $::tacaAdj + $::tjnaAdj + $::tvc1Adj + $::tjnbAdj + $::tacbAdj + $::tjpbAdj + $::tvc2Adj + $::tjpcAdj + $::taccAdj + $::tjncAdj]

    #set SamplingTime and 1/SamplingTime
    set std 1000
    set Ts 0.001

    set st1  [expr round($::tjpaAdj * $std) ] 
    set st2  [expr round($::tacaAdj * $std) ] 
    set st3  [expr round($::tjnaAdj * $std) ]
    set st4  [expr round($::tvc1Adj * $std) ]
    set st5  [expr round($::tjnbAdj * $std) ]
    set st6  [expr round($::tacbAdj * $std) ]
    set st7  [expr round($::tjpbAdj * $std) ]
    set st8  [expr round($::tvc2Adj * $std) ]
    set st9  [expr round($::tjpcAdj * $std) ]
    set st10 [expr round($::taccAdj * $std) ]
    set st11 [expr round($::tjncAdj * $std) ]


    set NOE [expr $st1 + $st2 + $st3 + $st4 + $st5 + $st6 + $st7 + $st8 + $st9 + $st10 + $st11 -1]

    # Building Vectors
    set Ts2 [expr $Ts * $Ts]
    set Ts3 [expr $Ts2 * $Ts]   
    set k 0 
    set ::st 0
    set ::vectJ 0
    set ::vectA $::a0
    set ::vectV $::v0
    set ::vectX 0

    for {set i 0} {$i < $NOE-1} {incr i 1} {
	lappend ::st $i
    }

    # Tjpa Interval 
    set ::Ai  $::a0
    set ::Vi  $::v0
    set ::Xi  $::x0
    if {  $st1 > 1 } {
	for {set i 0} {$i <= $st1} {incr i 1} {
	    incr k 1
	    set k1 [expr $Ts * $i]
	    set k2 [expr 0.5 * $Ts2 * $i *  $i] 
	    set k3 [expr   $i * $i * $i* $Ts3 / 6]
	    lappend ::vectJ [expr $::jmax]
	    lappend ::vectA [expr $::a0 + $k1 * $::jmax]
	    lappend ::vectV [expr $::v0 + $k1 * $::a0 + $k2 * $::jmax]
	    lappend ::vectX [expr $::x0 + $k1 * $::v0 + $k2 * $::a0 + $k3 * $::jmax]
	}
	set ::Ai [lindex $::vectA [expr $k]]
	set ::Vi [lindex $::vectV [expr $k]]
	set ::Xi [lindex $::vectX [expr $k]]
    }
    if {  $st2 > 1 } {
	# Taca Interval  
	for {set i 0} {$i <= $st2} {incr i 1} {
	    incr k 1
	    set k1 [expr $Ts * $i]
	    set k2 [expr 0.5 * $Ts2 * $i *  $i] 
	    lappend ::vectJ 0
	    lappend ::vectA $::Ai
	    lappend ::vectV [expr $::Vi + $k1 * $::Ai]
	    lappend ::vectX [expr $::Xi + $k1 * $::Vi + $k2 * $::Ai]	    
	}   
	set ::Ai [lindex $::vectA [expr $k]]
	set ::Vi [lindex $::vectV [expr $k]]
	set ::Xi [lindex $::vectX [expr $k]]
    }
    if {  $st3 > 1 } {
	# Tjna Interval  
	for {set i 1} {$i <= $st3} {incr i 1} {
	    incr k 1
	    set k1 [expr $Ts * $i]
	    set k2 [expr 0.5 * $Ts2 * $i *  $i] 
	    set k3 [expr   $i * $i * $i* $Ts3 / 6]
	    lappend ::vectJ [expr -$::jmax]
	    lappend ::vectA [expr $::Ai - $k1 * $::jmax] 
	    lappend ::vectV [expr $::Vi + $k1 * $::Ai - $k2 * $::jmax]
	    lappend ::vectX [expr $::Xi + $k1 * $::Vi + $k2 * $::Ai - $k3 * $::jmax]
	}   
	set ::Ai [lindex $::vectA [expr $k]]
	set ::Vi [lindex $::vectV [expr $k]]
	set ::Xi [lindex $::vectX [expr $k]]
    }
    if {  $st4 > 1 } {
	# Tvc1 Interval  
	for {set i 1} {$i <= $st4} {incr i 1} {
	    incr k 1
	    set k1 [expr $Ts * $i]
	    lappend ::vectJ 0
	    lappend ::vectA 0
	    lappend ::vectV [expr $::Vi]
	    lappend ::vectX [expr $::Xi + $k1 * $::Vi] 
	}   
	set ::Ai [lindex $::vectA [expr $k]]
	set ::Vi [lindex $::vectV [expr $k]]
	set ::Xi [lindex $::vectX [expr $k]]
    }

    if {  $st5 > 1 } {
	# Tjnb Interval  
	for {set i 1} {$i <= $st5} {incr i 1} {
	    incr k 1
	    set k1 [expr $Ts * $i]
	    set k2 [expr 0.5 * $Ts2 * $i *  $i] 
	    set k3 [expr   $i * $i * $i* $Ts3 / 6]
	    lappend ::vectJ [expr -$::jmax]
	    lappend ::vectA [expr $::Ai - $k1 * $::jmax] 
	    lappend ::vectV [expr $::Vi + $k1 * $::Ai - $k2 * $::jmax]
	    lappend ::vectX [expr $::Xi + $k1 * $::Vi + $k2 * $::Ai - $k3 * $::jmax]
	}   
	set ::Ai [lindex $::vectA [expr $k]]
	set ::Vi [lindex $::vectV [expr $k]]
	set ::Xi [lindex $::vectX [expr $k]]
    }
    if {  $st6 > 1 } {
	# Tacb Interval  
	for {set i 0} {$i <= $st6} {incr i 1} {
	    incr k 1
	    set k1 [expr $Ts * $i]
	    set k2 [expr 0.5 * $Ts2 * $i *  $i] 
	    lappend ::vectJ 0
	    lappend ::vectA $::Ai
	    lappend ::vectV [expr $::Vi + $k1 * $::Ai]
	    lappend ::vectX [expr $::Xi + $k1 * $::Vi + $k2 * $::Ai]	    
	}   
	set ::Ai [lindex $::vectA [expr $k]]
	set ::Vi [lindex $::vectV [expr $k]]
	set ::Xi [lindex $::vectX [expr $k]]
    }
    if {  $st7 > 1 } {
	# Tjpb Interval 
	for {set i 0} {$i <= $st7} {incr i 1} {
	    incr k 1
	    set k1 [expr $Ts * $i]
	    set k2 [expr 0.5 * $Ts2 * $i *  $i] 
	    set k3 [expr   $i * $i * $i* $Ts3 / 6]
	    lappend ::vectJ [expr $::jmax]
	    lappend ::vectA [expr $::Ai + $k1 * $::jmax]
	    lappend ::vectV [expr $::Vi + $k1 * $::Ai + $k2 * $::jmax]
	    lappend ::vectX [expr $::Xi + $k1 * $::Vi + $k2 * $::Xi + $k3 * $::jmax]
	}
	set ::Ai [lindex $::vectA [expr $k]]
	set ::Vi [lindex $::vectV [expr $k]]
	set ::Xi [lindex $::vectX [expr $k]]
    }

   if {  $st8 > 1 } {
	# Tvc2 Interval  
	for {set i 1} {$i <= $st8} {incr i 1} {
	    incr k 1
	    set k1 [expr $Ts * $i]
	    lappend ::vectJ 0
	    lappend ::vectA 0
	    lappend ::vectV [expr $::Vi]
	    lappend ::vectX [expr $::Xi + $k1 * $::Vi] 
	}   
	set ::Ai [lindex $::vectA [expr $k]]
	set ::Vi [lindex $::vectV [expr $k]]
	set ::Xi [lindex $::vectX [expr $k]]
     }
   if {  $st9 > 1 } {
       # Tjpc Interval  
       for {set i 0} {$i <= $st9} {incr i 1} {
           incr k 1
           set k1 [expr $Ts * $i]
           set k2 [expr 0.5 * $Ts2 * $i *  $i] 
           set k3 [expr   $i * $i * $i* $Ts3 / 6]
           lappend ::vectJ [expr $::jmax]
           lappend ::vectA [expr $::Ai + $k1 * $::jmax]
           lappend ::vectV [expr $::Vi + $k1 * $::Ai + $k2 * $::jmax]
           lappend ::vectX [expr $::Xi + $k1 * $::Vi + $k2 * $::Ai + $k3 * $::jmax]  
       }
       set ::Ai [lindex $::vectA [expr $k]]
       set ::Vi [lindex $::vectV [expr $k]]
       set ::Xi [lindex $::vectX [expr $k]]
    }
    if {  $st10 > 1 } {
       # Tacc Interval  
       for {set i 0} {$i <= $st10} {incr i 1} {
           incr k 1
           set k1 [expr $Ts * $i]
           set k2 [expr 0.5 * $Ts2 * $i *  $i]   
           lappend ::vectJ 0
           lappend ::vectA $::Ai
           lappend ::vectV [expr $::Vi + $k1 * $::Ai]
           lappend ::vectX [expr $::Xi + $k1 * $::Vi + $k2 * $::Ai]  
       }   
       set ::Ai [lindex $::vectA [expr $k]]
       set ::Vi [lindex $::vectV [expr $k]]
       set ::Xi [lindex $::vectX [expr $k]]
    }
   if {  $st11 > 1 } {
       # Tjnc Interval  
       for {set i 1} {$i <= $st11} {incr i 1} {
           incr k 1
           set k1 [expr $Ts * $i]
           set k2 [expr 0.5 * $Ts2 * $i *  $i] 
           set k3 [expr   $i * $i * $i* $Ts3 / 6]
           lappend ::vectJ [expr - $::jmax]
           lappend ::vectA [expr $::Ai - $k1 * $::jmax] 
           lappend ::vectV [expr $::Vi + $k1 * $::Ai - $k2 * $::jmax]
           lappend ::vectX [expr $::Xi + $k1 * $::Vi + $k2 * $::Ai - $k3 * $::jmax]   
       }   
    }
    set ::Af2 [lindex $::vectA [expr $k]]
    set ::Vf2 [lindex $::vectV [expr $k]]
    set ::Xf2 [lindex $::vectX [expr $k]]
 }
}

proc calcul_jerk_vc { args } {
    
    set ::x0 0
    $::limits configure -Jmax $::jmax -Amax $::amax -Vmax $::vmax
    $::jerkTimesAdjusted configure -timp $::timp
    $::jerkTimesAdjusted configure -Vc $::vcimp       
    $::ic configure -A $::a0 -V $::v0 -X $::x0
    $::fc configure -A $::af -V $::vf -X $::xf
    
    calculTimeProfileWithVcFixed $::limits $::ic $::fc $::jerkTimesAdjusted

    set ::t1Adj [format %.5f [jerkTimesAdjusted_t1_get $::jerkTimesAdjusted]]
    set ::t2Adj [format %.5f [jerkTimesAdjusted_t2_get $::jerkTimesAdjusted]]
    set ::t3Adj [format %.5f [jerkTimesAdjusted_t3_get $::jerkTimesAdjusted]]
    set ::t4Adj [format %.5f [jerkTimesAdjusted_t4_get $::jerkTimesAdjusted]]
    set ::t5Adj [format %.5f [jerkTimesAdjusted_t5_get $::jerkTimesAdjusted]]
    set ::t6Adj [format %.5f [jerkTimesAdjusted_t6_get $::jerkTimesAdjusted]]
    set ::t7Adj [format %.5f [jerkTimesAdjusted_t7_get $::jerkTimesAdjusted]]

    set ::dir_a [jerkTimesAdjusted_dir_a_get $::jerkTimesAdjusted]
    set ::dir_b [jerkTimesAdjusted_dir_b_get $::jerkTimesAdjusted]


    set ::totalTimeAdj [expr $::t1Adj + $::t2Adj + $::t3Adj + $::t4Adj + $::t5Adj + $::t6Adj + $::t7Adj]

    #set SamplingTime and 1/SamplingTime
    set std 500
    set Ts 0.002

    set st1 [expr round($::t1Adj * $std) ] 
    set st2 [expr round($::t2Adj * $std) ] 
    set st3 [expr round($::t3Adj * $std) ]
    set st4 [expr round($::t4Adj * $std) ]
    set st5 [expr round($::t5Adj * $std) ]
    set st6 [expr round($::t6Adj * $std) ]
    set st7 [expr round($::t7Adj * $std) ]
    set NOE [expr $st1 + $st2 + $st3 + $st4 + $st5 + $st6 + $st7 -1]

    # Building Vectors
    set Ts2 [expr $Ts * $Ts]
    set Ts3 [expr $Ts2 * $Ts]   
    set k 0 
    set ::st 0
    set ::vectJ 0
    set ::vectA $::a0
    set ::vectV $::v0
    set ::vectX 0

    for {set i 0} {$i < $NOE-1} {incr i 1} {
	lappend ::st $i
    }

    # Tjpa Interval 
    set ::Ai  $::a0
    set ::Vi  $::v0
    set ::Xi  $::x0
    if {  $st1 > 1 } {
	for {set i 0} {$i <= $st1} {incr i 1} {
	    incr k 1
	    set k1 [expr $Ts * $i]
	    set k2 [expr 0.5 * $Ts2 * $i *  $i] 
	    set k3 [expr   $i * $i * $i* $Ts3 / 6]
	    lappend ::vectJ [expr $::dir_a*$::jmax]
	    lappend ::vectA [expr $::a0 + $::dir_a * $k1 * $::jmax]
	    lappend ::vectV [expr $::v0 + $k1 * $::a0 + $::dir_a * $k2 * $::jmax]
	    lappend ::vectX [expr $::x0 + $k1 * $::v0 + $k2 * $::a0 + $::dir_a * $k3 * $::jmax]
	}
	set ::Ai [lindex $::vectA [expr $k]]
	set ::Vi [lindex $::vectV [expr $k]]
	set ::Xi [lindex $::vectX [expr $k]]
    }
    if {  $st2 > 1 } {
	# Taca Interval  
	for {set i 0} {$i <= $st2} {incr i 1} {
	    incr k 1
	    set k1 [expr $Ts * $i]
	    set k2 [expr 0.5 * $Ts2 * $i *  $i] 
	    lappend ::vectJ 0
	    lappend ::vectA $::Ai
	    lappend ::vectV [expr $::Vi + $k1 * $::Ai]
	    lappend ::vectX [expr $::Xi + $k1 * $::Vi + $k2 * $::Ai]	    
	}   
	set ::Ai [lindex $::vectA [expr $k]]
	set ::Vi [lindex $::vectV [expr $k]]
	set ::Xi [lindex $::vectX [expr $k]]
    }
    if {  $st3 > 1 } {
	# Tjna Interval  
	for {set i 1} {$i <= $st3} {incr i 1} {
	    incr k 1
	    set k1 [expr $Ts * $i]
	    set k2 [expr 0.5 * $Ts2 * $i *  $i] 
	    set k3 [expr   $i * $i * $i* $Ts3 / 6]
	    lappend ::vectJ [expr -$::dir_a*$::jmax]
	    lappend ::vectA [expr $::Ai - $::dir_a * $k1 * $::jmax] 
	    lappend ::vectV [expr $::Vi + $k1 * $::Ai - $::dir_a * $k2 * $::jmax]
	    lappend ::vectX [expr $::Xi + $k1 * $::Vi + $k2 * $::Ai - $::dir_a * $k3 * $::jmax]
	}   
	set ::Ai [lindex $::vectA [expr $k]]
	set ::Vi [lindex $::vectV [expr $k]]
	set ::Xi [lindex $::vectX [expr $k]]
    }
    if {  $st4 > 1 } {
	# Tvc Interval  
	for {set i 1} {$i <= $st4} {incr i 1} {
	    incr k 1
	    set k1 [expr $Ts * $i]
	    lappend ::vectJ 0
	    lappend ::vectA 0
	    lappend ::vectV [expr $::Vi]
	    lappend ::vectX [expr $::Xi + $k1 * $::Vi] 
	}   
	set ::Ai [lindex $::vectA [expr $k]]
	set ::Vi [lindex $::vectV [expr $k]]
	set ::Xi [lindex $::vectX [expr $k]]
    }
    if {  $st5 > 1 } {
	# Tjnb Interval  
	for {set i 1} {$i <= $st5} {incr i 1} {
	    incr k 1
	    set k1 [expr $Ts * $i]
	    set k2 [expr 0.5 * $Ts2 * $i *  $i] 
	    set k3 [expr   $i * $i * $i* $Ts3 / 6]
	    lappend ::vectJ [expr $::dir_b *$::jmax]
	    lappend ::vectA [expr $::Ai + $::dir_b * $k1 * $::jmax] 
	    lappend ::vectV [expr $::Vi + $k1 * $::Ai + $::dir_b * $k2 * $::jmax]
	    lappend ::vectX [expr $::Xi + $k1 * $::Vi + $k2 * $::Ai + $::dir_b * $k3 * $::jmax]   
	}   
	set ::Ai [lindex $::vectA [expr $k]]
	set ::Vi [lindex $::vectV [expr $k]]
	set ::Xi [lindex $::vectX [expr $k]]
    }
    if {  $st6 > 1 } {
	# Tacb Interval  
	for {set i 0} {$i <= $st6} {incr i 1} {
	    incr k 1
	    set k1 [expr $Ts * $i]
	    set k2 [expr 0.5 * $Ts2 * $i *  $i]   
	    lappend ::vectJ 0
	    lappend ::vectA $::Ai
	    lappend ::vectV [expr $::Vi + $k1 * $::Ai]
	    lappend ::vectX [expr $::Xi + $k1 * $::Vi + $k2 * $::Ai]  
	}   
	set ::Ai [lindex $::vectA [expr $k]]
	set ::Vi [lindex $::vectV [expr $k]]
	set ::Xi [lindex $::vectX [expr $k]]
    }
    if {  $st7 > 1 } {
	# Tjpb Interval  
	for {set i 0} {$i <= $st7} {incr i 1} {
	    incr k 1
	    set k1 [expr $Ts * $i]
	    set k2 [expr 0.5 * $Ts2 * $i *  $i] 
	    set k3 [expr   $i * $i * $i* $Ts3 / 6]
	    lappend ::vectJ [expr - $::dir_b * $::jmax]
	    lappend ::vectA [expr $::Ai - $::dir_b * $k1 * $::jmax]
	    lappend ::vectV [expr $::Vi + $k1 * $::Ai - $::dir_b * $k2 * $::jmax]
	    lappend ::vectX [expr $::Xi + $k1 * $::Vi + $k2 * $::Ai - $::dir_b * $k3 * $::jmax]  
	}
    }
    set ::Af2 [lindex $::vectA [expr $k]]
    set ::Vf2 [lindex $::vectV [expr $k]]
    set ::Xf2 [lindex $::vectX [expr $k]]

}

proc calcul_jerk_fond { args } {

    set ::a0fond 0
    set ::v0fond -$::vmax   
    $::limits configure -Jmax $::jmax -Amax $::amax -Vmax $::vmax
    $::icfond configure -A 0 -V -$::vmax -X $::x0
    $::fcfond configure -A 0 -V -$::vmax -X 1

    jerkTimes $::limits $::icfond $::fcfond $::jerkdatafond

    set ::t1fond [format %.5f [jerkData_t1_get $::jerkdatafond]]
    set ::t2fond [format %.5f [jerkData_t2_get $::jerkdatafond]]
    set ::t3fond [format %.5f [jerkData_t3_get $::jerkdatafond]]
    set ::t4fond [format %.5f [jerkData_t4_get $::jerkdatafond]]
    set ::t5fond [format %.5f [jerkData_t5_get $::jerkdatafond]]
    set ::t6fond [format %.5f [jerkData_t6_get $::jerkdatafond]]
    set ::t7fond [format %.5f [jerkData_t7_get $::jerkdatafond]]
    set ::dirfond [jerkData_dir_get $::jerkdatafond]

    #set SamplingTime and 1/SamplingTime
    set stdfond 1000
    set Tsfond 0.001
    set std 1000
    set Ts 0.001

    set st1fond [expr round($::t1fond * $stdfond) ] 
    set st2fond [expr round($::t2fond * $stdfond) ] 
    set st3fond [expr round($::t3fond * $stdfond) ]
    set st4fond [expr round($::t4fond * $stdfond) ]
    set st5fond [expr round($::t5fond * $stdfond) ]
    set st6fond [expr round($::t6fond * $stdfond) ]
    set st7fond [expr round($::t7fond * $stdfond) ]
    set NOEfond [expr $st1fond + $st2fond + $st3fond + $st4fond + $st5fond + $st6fond + $st7fond -1]

    # Building Vectors
    set Ts $Tsfond
    set Ts2 [expr $Tsfond * $Tsfond]
    set k 0 
    set ::vectJfond 0
    set ::vectAfond $::a0fond
    set ::vectVfond $::v0fond

    # Tjpa Interval 
    set ::Ai  $::a0fond
    set ::Vi  $::v0fond
    if {  $st1fond > 1 } {
	for {set i 0} {$i <= $st1fond} {incr i 1} {
	    incr k 1
	    set k1 [expr $Ts * $i]
	    set k2 [expr 0.5 * $Ts2 * $i *  $i] 
	    lappend ::vectJfond $::jmax
	    lappend ::vectAfond [expr $::a0fond + $::dirfond * $k1 * $::jmax]
	    lappend ::vectVfond [expr $::v0fond + $k1 * $::a0 + $::dirfond * $k2 * $::jmax]
	}
	set ::Ai [lindex $::vectAfond [expr $k]]
	set ::Vi [lindex $::vectVfond [expr $k]]
    }
    if {  $st2fond > 1 } {
	# Taca Interval  
	for {set i 0} {$i <= $st2fond} {incr i 1} {
	    incr k 1
	    set k1 [expr $Ts * $i]
	    lappend ::vectJfond 0
	    lappend ::vectAfond $::Ai
	    lappend ::vectVfond [expr $::Vi + $k1 * $::Ai]
	}   
	set ::Ai [lindex $::vectAfond [expr $k]]
	set ::Vi [lindex $::vectVfond [expr $k]]
    }
    if {  $st3fond > 1 } {
	# Tjna Interval  
	for {set i 1} {$i <= $st3fond} {incr i 1} {
	    incr k 1
	    set k1 [expr $Ts * $i]
	    set k2 [expr 0.5 * $Ts2 * $i *  $i] 
	    lappend ::vectJfond [expr -$::jmax]
	    lappend ::vectAfond [expr $::Ai - $::dirfond * $k1 * $::jmax] 
	    lappend ::vectVfond [expr $::Vi + $k1 * $::Ai - $::dirfond * $k2 * $::jmax]
	}   
	set ::Ai [lindex $::vectAfond [expr $k]]
	set ::Vi [lindex $::vectVfond [expr $k]]
    }
    if {  $st4fond > 1 } {
	# Tjvc Interval  
	for {set i 1} {$i <= $st4fond} {incr i 1} {
	    incr k 1
	    set k1 [expr $Ts * $i]
	    lappend ::vectJfond 0
	    lappend ::vectAfond 0
	    lappend ::vectVfond [expr $::Vi]
	}   
	set ::Ai [lindex $::vectAfond [expr $k]]
	set ::Vi [lindex $::vectVfond [expr $k]]
    }
    if {  $st5fond > 1 } {
	# Tjnb Interval  
	for {set i 1} {$i <= $st5fond} {incr i 1} {
	    incr k 1
	    set k1 [expr $Ts * $i]
	    set k2 [expr 0.5 * $Ts2 * $i *  $i] 
	    lappend ::vectJfond [expr -$::jmax]
	    lappend ::vectAfond [expr $::Ai - $::dirfond * $k1 * $::jmax] 
	    lappend ::vectVfond [expr $::Vi + $k1 * $::Ai - $::dirfond * $k2 * $::jmax]
	}   
	set ::Ai [lindex $::vectAfond [expr $k]]
	set ::Vi [lindex $::vectVfond [expr $k]]
    }
    if {  $st6fond > 1 } {
	# Tacb Interval  
	for {set i 0} {$i <= $st6fond} {incr i 1} {
	    incr k 1
	    set k1 [expr $Ts * $i]
	    lappend ::vectJfond 0
	    lappend ::vectAfond $::Ai
	    lappend ::vectVfond [expr $::Vi + $k1 * $::Ai]
	}   
	set ::Ai [lindex $::vectAfond [expr $k]]
	set ::Vi [lindex $::vectVfond [expr $k]]
    }
    if {  $st7fond > 1 } {
	# Tjpb Interval  
	for {set i 0} {$i <= $st7fond} {incr i 1} {
	    incr k 1
	    set k1 [expr $Ts * $i]
	    set k2 [expr 0.5 * $Ts2 * $i *  $i] 
	    lappend ::vectJfond $::jmax
	    lappend ::vectAfond [expr $::Ai + $::dirfond * $k1 * $::jmax]
	    lappend ::vectVfond [expr $::Vi + $k1 * $::Ai + $::dirfond * $k2 * $::jmax]
	}
    }
}
set lgth -0.1
proc calculForVideo { args } {

  if  {$::lgth >= 0.115} {
	  set ::lgth -0.1
	}
    set ::lgth [expr $::lgth + 0.001]
    set ::xf $::lgth
    update
    calcul_jerk
    puts "xf $::xf lgth $::lgth"
    graphique_jerk_times graph1
    graphique_jerk_vectors graph
   # after 100
    update
    if {$::lgth < 0.115} {
	    calculForVideo
	}

  
}


#  *------------------------------------------------------------*
#  * 
#  *                     --  Interface  --
#  * 
#  *------------------------------------------------------------*

catch {destroy $w}
toplevel $w
wm title $w "Jerk Times Viewer"

# Main frames
labelframe $w.limits -text "Parameters"
frame $w.conditions
frame $w.data
frame $w.but
pack  $w.limits $w.conditions $w.data $w.but -anchor n

# Frame $w.limits 
editval $w.limits.jmax -variable jmax label-text Jmax scale-length 200 -min 0 -max 1 -command {calcul_jerk;calcul_jerk_fond;graphique_jerk_times graph1; graphique_jerk_vectors graph } spinbox-increment 0.001

editval $w.limits.amax -variable amax label-text Amax scale-length 200 -min 0 -max 0.5 -command {calcul_jerk;calcul_jerk_fond;graphique_jerk_times graph1; graphique_jerk_vectors graph } spinbox-increment 0.001

editval $w.limits.vmax -variable vmax label-text Vmax scale-length 200 -min 0 -max 0.25 -command {calcul_jerk;calcul_jerk_fond;graphique_jerk_times graph1; graphique_jerk_vectors graph } spinbox-increment 0.001
pack $w.limits.jmax $w.limits.amax $w.limits.vmax -anchor e

# Frame $w.conditions
labelframe $w.conditions.initial -text "Initial Conditions"
editval $w.conditions.initial.a -variable a0 label-text A0 scale-length 200 -min -0.2 -max 0.2 -command {calcul_jerk;graphique_jerk_times graph1; graphique_jerk_vectors graph } spinbox-increment 0.001
editval $w.conditions.initial.v -variable v0 label-text V0 scale-length 200 -min -0.1 -max 0.1 -command {calcul_jerk;graphique_jerk_times graph1; graphique_jerk_vectors graph } spinbox-increment 0.001
editval $w.conditions.initial.x -variable x0 label-text X0 scale-length 200 -min 0 -max 0

labelframe $w.conditions.final -text "Final Conditions"
editval $w.conditions.final.a -variable af label-text Af scale-length 200 -min -0.2 -max 0.2 -command {calcul_jerk;graphique_jerk_times graph1; graphique_jerk_vectors graph } spinbox-increment 0.001 
editval $w.conditions.final.v -variable vf label-text Vf scale-length 200 -min -0.1 -max 0.1 -command {calcul_jerk;graphique_jerk_times graph1; graphique_jerk_vectors graph } spinbox-increment 0.001
editval $w.conditions.final.x -variable xf label-text Xf scale-length 200 -min -0.5 -max 0.5 -command {calcul_jerk;graphique_jerk_times graph1; graphique_jerk_vectors graph } spinbox-increment 0.001

labelframe $w.conditions.adjustTime -text "Adjust Time"
editval $w.conditions.adjustTime.timeImp -variable timp label-text "Time Imp." scale-length 200 -min 0 -max 5  -command {calcul_jerk_adjusted;graphique_jerk_times graph1; graphique_jerk_vectors graph } spinbox-increment 0.025
editval $w.conditions.adjustTime.vcImp -variable vcimp label-text "Vc" scale-length 200 -min -0.1 -max 0.1  -command {calcul_jerk_vc;graphique_jerk_times graph1; graphique_jerk_vectors graph }  spinbox-increment 0.025

pack  $w.conditions.initial $w.conditions.final $w.conditions.adjustTime -anchor n  -pady 7
pack  $w.conditions.initial.a $w.conditions.initial.v $w.conditions.initial.x
pack  $w.conditions.final.a  $w.conditions.final.v  $w.conditions.final.x
pack  $w.conditions.adjustTime.timeImp $w.conditions.adjustTime.vcImp

# Frame  $w.data
labelframe $w.data.t
labelframe $w.data.tAdjusted
labelframe $w.data.d
pack  $w.data.t $w.data.tAdjusted $w.data.d -side left -anchor n -padx 10

# Frame  $w.data.t
label  $w.data.t.dir -textvariable chemin 
frame  $w.data.t.times
frame  $w.data.t.tlabel
pack  $w.data.t.dir 
pack $w.data.t.tlabel $w.data.t.times -anchor n -side left
label $w.data.t.times.ft1  -textvariable t1 -relief sunken
label $w.data.t.times.ft2  -textvariable t2 -relief sunken
label $w.data.t.times.ft3  -textvariable t3 -relief sunken
label $w.data.t.times.ft4  -textvariable t4 -relief sunken
label $w.data.t.times.ft5  -textvariable t5 -relief sunken
label $w.data.t.times.ft6  -textvariable t6 -relief sunken
label $w.data.t.times.ft7  -textvariable t7 -relief sunken
label $w.data.t.times.ftt  -textvariable totalTime -background "#0066FF" -foreground "#FFFFFF" 
label $w.data.t.tlabel.lt1 -text "T1 (s)"
label $w.data.t.tlabel.lt2 -text "T2 (s)"
label $w.data.t.tlabel.lt3 -text "T3 (s)"
label $w.data.t.tlabel.lt4 -text "T4 (s)"
label $w.data.t.tlabel.lt5 -text "T5 (s)"
label $w.data.t.tlabel.lt6 -text "T6 (s)"
label $w.data.t.tlabel.lt7 -text "T7 (s)"
label $w.data.t.tlabel.ltt -text "TT (s)"

pack  $w.data.t.times.ft1 $w.data.t.times.ft2 $w.data.t.times.ft3 $w.data.t.times.ft4 $w.data.t.times.ft5 $w.data.t.times.ft6 $w.data.t.times.ft7 $w.data.t.times.ftt -anchor n -pady 1
pack $w.data.t.tlabel.lt1 $w.data.t.tlabel.lt2 $w.data.t.tlabel.lt3 $w.data.t.tlabel.lt4 $w.data.t.tlabel.lt5 $w.data.t.tlabel.lt6 $w.data.t.tlabel.lt7 $w.data.t.tlabel.ltt  -anchor e -pady 1

# Frame  $w.data.tAdjusted
label  $w.data.tAdjusted.text -text "Adjusted Times"
frame  $w.data.tAdjusted.times
frame  $w.data.tAdjusted.tlabel
pack  $w.data.tAdjusted.text 
pack $w.data.tAdjusted.tlabel $w.data.tAdjusted.times -anchor n -side left
label $w.data.tAdjusted.times.ft1  -textvariable t1Adj -relief sunken
label $w.data.tAdjusted.times.ft2  -textvariable t2Adj -relief sunken
label $w.data.tAdjusted.times.ft3  -textvariable t3Adj -relief sunken
label $w.data.tAdjusted.times.ft4  -textvariable t4Adj -relief sunken
label $w.data.tAdjusted.times.ft5  -textvariable t5Adj -relief sunken
label $w.data.tAdjusted.times.ft6  -textvariable t6Adj -relief sunken
label $w.data.tAdjusted.times.ft7  -textvariable t7Adj -relief sunken
label $w.data.tAdjusted.times.ftt  -textvariable totalTimeAdj -background "#0066FF" -foreground "#FFFFFF" 
label $w.data.tAdjusted.tlabel.lt1 -text "T1 (s)"
label $w.data.tAdjusted.tlabel.lt2 -text "T2 (s)"
label $w.data.tAdjusted.tlabel.lt3 -text "T3 (s)"
label $w.data.tAdjusted.tlabel.lt4 -text "T4 (s)"
label $w.data.tAdjusted.tlabel.lt5 -text "T5 (s)"
label $w.data.tAdjusted.tlabel.lt6 -text "T6 (s)"
label $w.data.tAdjusted.tlabel.lt7 -text "T7 (s)"
label $w.data.tAdjusted.tlabel.ltt -text "TT (s)"

pack  $w.data.tAdjusted.times.ft1 $w.data.tAdjusted.times.ft2 $w.data.tAdjusted.times.ft3 $w.data.tAdjusted.times.ft4 $w.data.tAdjusted.times.ft5 $w.data.tAdjusted.times.ft6 $w.data.tAdjusted.times.ft7 $w.data.tAdjusted.times.ftt -anchor n -pady 1
pack $w.data.tAdjusted.tlabel.lt1 $w.data.tAdjusted.tlabel.lt2 $w.data.tAdjusted.tlabel.lt3 $w.data.tAdjusted.tlabel.lt4 $w.data.tAdjusted.tlabel.lt5 $w.data.tAdjusted.tlabel.lt6 $w.data.tAdjusted.tlabel.lt7 $w.data.tAdjusted.tlabel.ltt  -anchor e -pady 1

# Frame  $w.data.d
label $w.data.d.fdc  -textvariable dc -relief sunken
label $w.data.d.ldc -text "Critical Length (m)"
label $w.data.d.fzone  -textvariable zone -relief sunken
label $w.data.d.lzone -text "Type de zone"
pack   $w.data.d.ldc $w.data.d.fdc $w.data.d.lzone $w.data.d.fzone -anchor n

# Frame  $w.but
#button $w.but.calculJerk -text Calcul -command {calcul_jerk } -background \#ffff66  -activebackground \#66ff00 
button $w.but.calculJerk -text Calcul -command { calculForVideo } -background \#ffff66  -activebackground \#66ff00 
#button $w.but.calculJerk -text Calcul -command {  calcul_jerk_adjustedVc1Vc2; graphique_jerk_times graph1; graphique_jerk_vectors graph  } -background \#ffff66  -activebackground \#66ff00


button $w.but.graphiqueJerkAll -text "Plot All" -command {calcul_jerk;graphique_jerk_times graph1;graphique_jerk_vectors graph } -background \#ffff66  -activebackground \#66ff00 
button $w.but.graphiqueJerkTimes -text "Plot Times" -command {calcul_jerk;graphique_jerk_times graph1 } -background \#ffff66  -activebackground \#66ff00 
button $w.but.graphiqueJerkVectors -text "Plot Vectors" -command {calcul_jerk;graphique_jerk_vectors graph } -background \#ffff66  -activebackground \#66ff00 
button $w.but.exit -text Exit -command {destroy $w;destroy  .graph;destroy  .graph1} -background \#ffff66  -activebackground \#66ff00 
pack $w.but.calculJerk $w.but.graphiqueJerkVectors $w.but.graphiqueJerkTimes $w.but.graphiqueJerkAll  -side left 
pack $w.but.exit -anchor e
