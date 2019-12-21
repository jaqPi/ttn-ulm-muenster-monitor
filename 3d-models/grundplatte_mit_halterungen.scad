// Grundplatte mit Halterungen, für den Sensor des Münster-Monitorings

// mm

// X Position x-Achse
// Y Position y-Achse
// B Breite x-Achse
// H Höhe   z-Achse
// T Tiefe  y-Achse
// R Radius
// D Durchmesser


$fa = 0.1;
$fs = 0.1;

// PARAMETRIERUNG

e           = 0.4;  // Luft für Ungenauigkeiten des Druckers.


//Grundplatte gp mit Bohrungen:
gpUrsprX    = 0;
gpUrsprY    = 0;

gpB         = 70;
gpH         = 70;
gpT         = 6.2; //6.2

gpBohrungAktiv =  true;
gpBohrungX  = 0;
gpBohrungY  = 0;
gpBohrungD  = 5;
gpBohrungB  = gpB;//gpB - 2 * gpBohrungX;
gpBohrungH  = gpH;//gpH - 2 * gpBohrungY;


// Pins für Mikro-Controller pc:
pcX         = 20;  // Position Mittelpunkt Pin links unten
pcY         = 10;
// Maße für den Adafruit Feather:
pcB         = 45.6;  // Abstand Mittelpunkte Pins in x-Richtung
pcH         = 17.6;  // Abstand Mittelpunkte Pins in x-Richtung

pcBasisD    = 4;
pcBasisT    = 7 + 4 + gpT; // 7 mm für Dicke Akku, 4 mm Luft
pcPinD      = 2.5 - 1.5 * e;
pcPinT      = 4  + pcBasisT;  // Dicke PCB Feather ~ 1,6 mm



// Röhre als Durchlass r:

rH          = 12;
rDAussen    = 14;
rDickeWand  = 1;
rBasisD     = 4 + rDAussen;
rBasisH     = 1.0;
rHoeheUnterkanteBasisDUeberGP = 0;

// Scheibe zum Abschluss der Röhre s:

sDicke      = 1;
rHalterungBasisH = rBasisH + sDicke + 1.5 * e;
rHalterungBasisH = sDicke + 1.5 * e;


// Break out boards bob:

bobHoeheUnterkanteUeberGP = 4;
bobSteckerT = 2.6;
babAbstaendeInHalterung = 2;


// BME280, Billo-Breakoutboard bobBME280:

bobBME280Breite             = 10.5 + e;
bobBME280Dicke              = 1.55 + e; // Dicke PCB bobBME280 ~ 1,57 mm
bobBME280HoeheBisStecker    = 11;
bobBME280TiefeFuge          = 1 - e;
bobVME280X                  = 4;      // relativ zum Sensorhalterblock
//bobVME280Y                = bobVL6180XY + bobVL6180XBreite + babAbstaendeInHalterung;
bobVME280Y                  = babAbstaendeInHalterung + 0;
bobVME280Z                  = gpT + bobHoeheUnterkanteUeberGP - (gpT - 1) - 1;


// VL6180X, Pololu-Breakoutboard bobVL6180X:

bobVL6180XBreite            = 17.8 + e;
bobVL6180XDicke             = 0.9 + e;
bobVL6180XHoeheBisStecker   = 10;
bobVL6180XHoehe             = bobVL6180XHoeheBisStecker + bobSteckerT;
bobVL6180XTiefeFuge         = 1 - e ;       // Dicke PCB bobVL6180X ~ 1 mm
bobVL6180XX                 = 9;
//bobVL6180XY               = babAbstaendeInHalterung + 0;
bobVL6180XY                 = bobVME280Y + bobBME280Breite + babAbstaendeInHalterung;
bobVL6180XZ                 = gpT + bobHoeheUnterkanteUeberGP - (gpT - 1);


// Block, um in der Sensorhalterung Aussparungen für die Stecker zu schaffen

mBreakoutboardSteckerBlockT = 10;
mBreakoutboardSteckerBlockB = 20;
mBreakoutboardSteckerBlockOffsetX = -2; // Für Überstand Pins/Lötkegel


// Block für Halterung der Sensoren sb:

sbB     = 11;
sbH     = bobVL6180XBreite + bobBME280Breite + 3 * babAbstaendeInHalterung;
sbT     = gpT + bobHoeheUnterkanteUeberGP + bobVL6180XHoehe - (gpT - 1);
sbX     = 0;
sbY     = 48 - ( 2 * ( 2 + e ) + bobBME280Breite + (rBasisD - rDAussen) / 2);
sbZ     = gpT - 0.2;






// AUFBAU


// Grundplatte mit Pins:

module bohrung (x, y, z = 0,
                h = gpT * 4, d = gpBohrungD, centered = true
                ) {
    translate([x, y, z]) {
        cylinder(h = h, d = d, center = centered);
    }
}

module pin (x, y, z = 0) {
    translate([x, y, z]) {
        union () {
            cylinder(h = pcBasisT, d = pcBasisD);
            cylinder(h = pcPinT, d = pcPinD);
        }
    }
}


difference () {

    cube([gpB, gpH, gpT]);

    if (gpBohrungAktiv) {
        bohrung(gpBohrungX             , gpBohrungY);
        bohrung(gpBohrungX + gpBohrungB, gpBohrungY);
        bohrung(gpBohrungX             , gpBohrungY + gpBohrungH);
        bohrung(gpBohrungX + gpBohrungB, gpBohrungY + gpBohrungH);
    }
    
    bohrung(x = 35, y = 1, d = 10 + 3 * e);
    bohrung(x = 35, y = 70 - 1, d = 10 + 3 * e);
    translate([35, 0, 0]) {
        cube([10 + 2 * e, (10 + 3 * e) / 2, gpT * 4], center = true);
    }
    translate([35, 70, 0]) {
        cube([10 + 2 * e, (10 + 3 * e) / 2, gpT * 4], center = true);
    }
    
    bohrung(x = 35, y = 5 + 6.8,          d = 15 + e);
    bohrung(x = 35, y = 70 - (5 + 6.8),   d = 15 + e);
    
    bohrung(x = 35, y = 35 - 14, d = 7.5 + 3 * e);
    bohrung(x = 35, y = 35 + 14, d = 7.5 + 3 * e);
    
    bohrung(x = 35,         y = 35, d = 10.2 + 2 * e);
    bohrung(x = 35 - 26,    y = 35, d = 10.2 + 2 * e);
    
    translate ([14, 14 ,-1]) {
        cube([12, 13, 2 + 1]);
        translate ([0, 29 , 0]) {
            cube([12, 13, 2 + 1]);
        }
    }
    
}

pin(pcX      , pcY);
pin(pcX + pcB, pcY);
pin(pcX      , pcY + pcH);
pin(pcX + pcB, pcY + pcH);



// Röhre

module roehre ( x, y, z, 
                hoehe,
                durchmesserAussen,
                dickeWand,
                basisDurchmesser,
                basisHoehe,
                gefuellt = false,
                basis = true) {

    zentriert = true;
    translate([x, y, z]) {
        difference () {
            union () {
                translate ([0, 0, hoehe / 2]) {
                    cylinder(h = hoehe, d = durchmesserAussen, center = zentriert);
                }
                if (basis) {
                    translate ([0, 0, basisHoehe / 2]) {
                        cylinder(h = basisHoehe, d = basisDurchmesser, center = zentriert);
                    }
                }
            }
            if (! gefuellt) {
                cylinder(h = hoehe * 4, d = durchmesserAussen - 2 * dickeWand, center = zentriert);
            }
        }
    }
}



module roehreAussparung () {
    translate ([0, rBasisD / 2, rBasisD / 2 + rHoeheUnterkanteBasisDUeberGP]) {
    union () {
        linear_extrude (20) {
            projection () {
                rotate ([0, -90, 0 ]) {
                    roehre (0, 0, 0,
                            rH, rDAussen, rDickeWand, rBasisD, rHalterungBasisH,
                            gefuellt = true);
                }
            }
        }
        rotate ([0, -90, 0 ]) {
            roehre (0, 0, 0,
                    rH, rDAussen, rDickeWand, rBasisD, rHalterungBasisH,
                    gefuellt = true);
        }
    }
}
}

roehre (rBasisD / 2, gpH + rBasisD / 2 + 1 , 0,
        rH, rDAussen, rDickeWand, rBasisD, rBasisH,
        gefuellt = false, basis = false);

//roehre (rBasisD * 1.5, gpH + rBasisD / 2 + 1 , 0,
//        rH, rDAussen, rDickeWand, rBasisD, rBasisH,
//        gefuellt = false);




// Sensorhalterung

module breakoutboard (breite, dicke, hoeheBisStecker, breiteFuge, x, y, z) {
    translate ([x, y, z]) {
    union () {
        cube([dicke, breite, hoeheBisStecker+mBreakoutboardSteckerBlockT]);
        translate ([mBreakoutboardSteckerBlockOffsetX, 0, hoeheBisStecker]) {
            cube([mBreakoutboardSteckerBlockB, breite, mBreakoutboardSteckerBlockT]);
        }
        translate ([mBreakoutboardSteckerBlockOffsetX, breiteFuge, 0]) {
            cube([  mBreakoutboardSteckerBlockB,
                    breite - 2 * breiteFuge,
                    hoeheBisStecker + mBreakoutboardSteckerBlockT
            ]);
        }
    }
    }
}


translate([sbX, sbY, sbZ]) {
difference () {
    
    cube([sbB, sbH, sbT]);
    
    breakoutboard(bobVL6180XBreite, bobVL6180XDicke, bobVL6180XHoeheBisStecker, bobVL6180XTiefeFuge, bobVL6180XX, bobVL6180XY, bobVL6180XZ);
    
    breakoutboard(bobBME280Breite, bobBME280Dicke, bobBME280HoeheBisStecker, bobBME280TiefeFuge, bobVME280X, bobVME280Y, bobVME280Z);    
    
    translate ([bobVL6180XX - ( 1 + abs(mBreakoutboardSteckerBlockOffsetX)) ,
                bobVL6180XY + (bobVL6180XBreite - rBasisD) / 2 ,
                1
                ]) {
        roehreAussparung ();
        translate ([-rHalterungBasisH, rBasisD, 0]) {
            rotate ([180, 180, 0]) {
                roehreAussparung ();
            }
        }

    }
    
    // Materialeinsparungen:
    translate([-1 , bobVME280Y - babAbstaendeInHalterung - e, -1]) {
        cube([  bobVME280X + mBreakoutboardSteckerBlockOffsetX + 2.01,
                bobBME280Breite + babAbstaendeInHalterung + 0.01 + e,
                sbT * 2
            ]);
        translate([8, 0, -1]) {
            cube([  bobVME280X + mBreakoutboardSteckerBlockOffsetX + 3.01,
                bobBME280Breite + babAbstaendeInHalterung + 0.01 + e,
                sbT * 2
            ]);
        }
    }
    
}
}
