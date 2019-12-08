// Grundplatte mit Halterungen, für den Sensor des Münster-Monitorings

// mm

// X Position x-Achse
// Y Position y-Achse
// B Breite x-Achse
// H Höhe   z-Achse
// T Tiefe  y-Achse
// R Radius
// D Durchmesser



// PARAMETRIERUNG


//Grundplatte gp:
gpUrsprX    = 0;
gpUrsprY    = 0;

gpB         = 70;
gpH         = 70;
gpT         = 1;

gpBohrungX  = 5;
gpBohrungY  = 5;
gpBohrungD  = 1.5;
gpBohrungB  = gpB - 2 * gpBohrungX;
gpBohrungH  = gpH - 2 * gpBohrungY;


// Pins für Mikro-Controller pc:
pcX         = 40;  // Position Mittelpunkt Pin links unten
pcY         = 20;
pcB         = 18;  // Abstand Mittelpunkte Pins in x-Richtung
pcH         = 45;  // Abstand Mittelpunkte Pins in x-Richtung

pcBasisD    = 4;
pcBasisT    = 2 + gpT;
pcPinD      = 2.5;
pcPinT      = 2 + pcBasisT;



// Röhre als Durchlass r:

rH          = 20;
rDAussen    = 10;
rDickeWand  = 1;
rBasisD     = 4 + rDAussen;
rBasisH     = 1.5;


// Scheibe zum Abschluss der Röhre s:

sDicke      = 0.5;
rHalterungBasisH = rBasisH + sDicke;


// Break out boards bob:

bobHoeheUnterkanteUberGP = 2;
bobSteckerT = 2.54;
babAbstaendeInHalterung = 2;


// VL6180X, Pololu-Breakoutboard bobVL6180X:

bobVL6180XBreite = 18;
bobVL6180XDicke = 1;
bobVL6180XHoeheBisStecker = 10;
bobVL6180XHoehe = bobVL6180XHoeheBisStecker + bobSteckerT;
bobVL6180XBreiteFuge = 1;
bobVL6180XX = 9;
bobVL6180XY = babAbstaendeInHalterung;
bobVL6180XZ = gpT + bobHoeheUnterkanteUberGP;


// BME280, Billo-Breakoutboard bobBME280:

bobBME280Breite = 10;
bobBME280Dicke = 1;
bobBME280HoeheBisStecker = 10;
bobBME280BreiteFuge = 1;
bobVME280X = 9;     // relativ zum Sensorhalterblock
bobVME280Y = bobVL6180XY + bobVL6180XBreite + babAbstaendeInHalterung;
bobVME280Z = gpT + bobHoeheUnterkanteUberGP;

// Block, um in der Sensorhalterung Aussparungen für die Stecker zu schaffen

mBreakoutboardSteckerBlockT = 10;
mBreakoutboardSteckerBlockB = 20;
mBreakoutboardSteckerBlockOffsetX = -1.5; // Für Überstand Pins/Lötkegel


// Block für Halterung der Sensoren sb:

sbB     = 11;
sbH     = bobVL6180XBreite + bobBME280Breite + 3 * babAbstaendeInHalterung;
sbT     = gpT + bobHoeheUnterkanteUberGP + bobVL6180XHoehe;
sbX     = 0;
sbY     = 20;
sbZ     = 0;






// AUFBAU


// Grundplatte mit Pins:

module bohrung (x, y, z = 0) {
    translate([x, y, z]) {
        cylinder(h = gpT * 4, d = gpBohrungD, center = true);
    }
}

module pin (x, y, z = 0) {
    translate([x, y, 0]) {
        union () {
            cylinder(h = pcBasisT, d = pcBasisD);
            cylinder(h = pcPinT, d = pcPinD);
        }
    }
}


difference () {
    cube([gpB, gpH, gpT]);
    bohrung(gpBohrungX             , gpBohrungY);
    bohrung(gpBohrungX + gpBohrungB, gpBohrungY);
    bohrung(gpBohrungX             , gpBohrungY + gpBohrungH);
    bohrung(gpBohrungX + gpBohrungB, gpBohrungY + gpBohrungH);
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
                gefuellt = false) {
    zentriert = true;
    translate([x, y, z]) {
        difference () {
            union () {
                translate ([0, 0, hoehe / 2]) {
                    cylinder(h = hoehe, d = durchmesserAussen, center = zentriert);
                }
                translate ([0, 0, basisHoehe / 2]) {
                    cylinder(h = basisHoehe, d = basisDurchmesser, center = zentriert);
                }
            }
            if (! gefuellt) {
                cylinder(h = hoehe * 4, d = durchmesserAussen - 2 * dickeWand, center = zentriert);
            }
        }
    }
}


module roehreAussparung () {
    translate ([0, rBasisD / 2, rBasisD / 2]) {
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
        gefuellt = false);



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
    
    breakoutboard(bobVL6180XBreite, bobVL6180XDicke, bobVL6180XHoeheBisStecker, bobVL6180XBreiteFuge, bobVL6180XX, bobVL6180XY, bobVL6180XZ);
    
    breakoutboard(bobBME280Breite, bobBME280Dicke, bobBME280HoeheBisStecker, bobBME280BreiteFuge, bobVME280X, bobVME280Y, bobVME280Z);    
    
    translate ([bobVL6180XX - ( 1 + abs(mBreakoutboardSteckerBlockOffsetX)) ,
                bobVL6180XY + (bobVL6180XBreite - rBasisD) / 2,
                bobVL6180XZ]) {
        roehreAussparung ();
        translate ([-rHalterungBasisH, rBasisD, 0]) {
            rotate ([180, 180, 0]) {
                roehreAussparung ();
            }
        }

    }
    
    // Materialeinsparungen:
    translate([-1 , bobVME280Y, gpT]) {
        cube([  bobVME280X + mBreakoutboardSteckerBlockOffsetX + 1.01,
                bobBME280Breite + babAbstaendeInHalterung + 0.01,
                sbT
            ]);
    }
    
}
}

