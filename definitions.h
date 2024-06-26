
//#define CVIN_PIN A0
//#define SCALE_PIN A2
//#define KEY_OF_PIN A7
#define CHANGE_LAYOUT_PIN PB0
#define CHANGE_CV_MODE_PIN PB1
#define TRIGGER_OUT_PIN PB2


#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
//#define OLED_RESET     4 // Reset pin # (or -1 if sharing Arduino reset pin)

/*
#define MAX_DAC_SEMITONE 61
const int16_t PROGMEM semitone_cvs_dac[MAX_DAC_SEMITONE] = {
// 0     83.3  166.6  250   333   416.6  500  583.3  666.6  750   833.3  916.6
  6,     65,   131,   201,  268,  337,   405,  474,  540,   609,  677,   745,
  814,   882,  950,  1016, 1084, 1151,  1223, 1290, 1359,  1427, 1496,  1563, 
  1631, 1699, 1768,  1838, 1904, 1974,  2041, 2110, 2177,  2248, 2315,  2386,
  2455, 2524, 2592,  2660, 2729, 2798,  2867, 2934, 3004,  3071, 3141,  3206,
  3278, 3344, 3412,  3480, 3549, 3617,  3685, 3754, 3822,  3892, 3960,  4026,
  4093
};
*/
#define MAX_DAC_SEMITONE 121
const int16_t PROGMEM semitone_cvs_dac[MAX_DAC_SEMITONE] = {
    0, 1, 2, 3, 4, 5, 6, 7, 8, 9,
    10, 11, 12, 13, 14, 15, 16, 17, 18, 19,
    20, 21, 22, 23, 24, 25, 26, 27, 28, 29,
    30, 31, 32, 33, 34, 35, 36, 37, 38, 39,
    40, 41, 42, 43, 44, 45, 46, 47, 48, 49,
    50, 51, 52, 53, 54, 55, 56, 57, 58, 59,
    60, 61, 62, 63, 64, 65, 66, 67, 68, 69,
    70, 71, 72, 73, 74, 75, 76, 77, 78, 79,
    80, 81, 82, 83, 84, 85, 86, 87, 88, 89,
    90, 91, 92, 93, 94, 95, 96, 97, 98, 99,
    100, 101, 102, 103, 104, 105, 106, 107, 108, 109,
    110, 111, 112, 113, 114, 115, 116, 117, 118, 119,
    120
};



// 0  83  167  250  333  417  500  583  667  750  833  917
// Techically the ADC can distinguish amongst 63 semitones, but the DAC can
// only produce 57 because of the power of my USB port

//#define CV_0V_BOUNDARY_INCLUSIVE -32
//#define CV_ABOVE_5V_BOUNDARY_EXCLUSIVE 26832
//#define NB_ADC_BOUNDARIES 62
#define CV_0V_BOUNDARY_INCLUSIVE -17
#define CV_ABOVE_5V_BOUNDARY_EXCLUSIVE 4063
#define NB_ADC_BOUNDARIES 122


// ADS1015 maps 6.14 at 32767, but we stop at 5V to stay within standards.
// These are the inter CVs ADC reading, i.e. we round CVs by placing boundaries
// at 1/24V left and right of the CV associated with a semitone,
// in other words if you are within k*83.3mV ± 41.6mV then bin CV into k
/*
const int16_t PROGMEM inter_semitonecv_to_ADC16read[NB_ADC_BOUNDARIES] = {
        -240,   208,   656,  1104,  1544,  1984,  2432,  2880,  3328,  3776,  4216,  4656,
        5096,  5544,  5992,  6432,  6872,  7320,  7776,  8216,  8648,  9088,  9536,  9984,
       10432, 10880, 11320, 11760, 12208, 12648, 13088, 13536, 13984, 14424, 14864, 15312,
       15752, 16200, 16656, 17104, 17552, 17992, 18432, 18872, 19312, 19760, 20200, 20648,
       21096, 21536, 21976, 22416, 22872, 23312, 23752, 24200, 24648, 25096, 25536, 25984, 
       26432, // 26880,
       32765
};
*/
const int16_t PROGMEM inter_semitonecv_to_ADC16read[NB_ADC_BOUNDARIES] = {
    -17, 17, 51, 85, 119, 153, 187, 221, 255, 289,
    323, 357, 391, 425, 459, 493, 527, 561, 595, 629,
    663, 697, 731, 765, 799, 833, 867, 901, 935, 969,
    1003, 1037, 1071, 1105, 1139, 1173, 1207, 1241, 1275, 1309,
    1343, 1377, 1411, 1445, 1479, 1513, 1547, 1581, 1615, 1649,
    1683, 1717, 1751, 1785, 1819, 1853, 1887, 1921, 1955, 1989,
    2023, 2057, 2091, 2125, 2159, 2193, 2227, 2261, 2295, 2329,
    2363, 2397, 2431, 2465, 2499, 2533, 2567, 2601, 2635, 2669,
    2703, 2737, 2771, 2805, 2839, 2873, 2907, 2941, 2975, 3009,
    3043, 3077, 3111, 3145, 3179, 3213, 3247, 3281, 3315, 3349,
    3383, 3417, 3451, 3485, 3519, 3553, 3587, 3621, 3655, 3689,
    3723, 3757, 3791, 3825, 3859, 3893, 3927, 3961, 3995, 4029,
    4063, 4097
};




#define NUM_NOTES 12
#define NUM_SCALES 120

#define KEYBOARD_WIDTH 11
#define SCALE_BITS 12

#define WKEY_WIDTH 7
#define WKEY_HEIGHT 12

#define BKEY_WIDTH 5
#define BKEY_HEIGHT 10

#define KEYBOARD_LAYOUT_WIDTH 56
#define KEYBOARD_LAYOUT_HEIGHT 16

// Scales with note 1 in least significant bit
//                  flat-2 in bit 1
//                  2 in bit 2
// and 7 in most significant bit 11.

//    1 f2  2 f3  3  4 f5  5 f6  6 f7  7
//bit 0  1  2  3  4  5  6  7  8  9 10 11

// Main scales with 1st note at LSB
#define CHROMATIC 0xfff
#define MAJOR 0xab5
#define MINOR 0x5ad
#define HARMONIC_MINOR 0x9ad
#define MAJOR_PENTATONIC 0x295
#define MINOR_PENTATONIC 0x4a9
#define BLUES 0x4e9
#define JAPANESE 0x18d
#define FREYGISH 0x5b3
#define ROMANI 0x9cd
#define ARABIC 0x9b3
#define ALTERED 0x55b
#define WHOLE_TONE 0x555
#define HW_DIMINISHED 0x6db
#define WH_DIMINISHED 0xb6d

#define IONIAN 0xab5
#define DORIAN 0x6ad
#define PHRYGIAN 0x5ab
#define LYDIAN 0xad5
#define MIXOLYDIAN 0x6b5
#define AEOLIAN 0x5ad
#define LOCRIAN 0x56b
#define IONIAN_B2 0xab3
#define DORIAN_B5 0x66d
#define HARMONIC_PHRYGIAN 0x9ab
#define PHRYGIAN_MAJOR 0xaad
#define LYDIAN_B3 0xacd
#define MAJOR_LOCRIAN 0x575
#define MINOR_LOCRIAN 0x56d
#define SUPER_LOCRIAN 0x55b

#define LYDIAN_7B 0x6d5
//#define ALTERED
#define DIMINISHED 0x6db
#define MIXOLYDIAN_B13 0x5b5
#define MIXOLYDIAN_B9_B13 0x5b3
#define LYDIAN_7B_B2 0x6ab
#define BEBOP 0x6b5
//#define WHOLE_TONE
#define BLUES_MAJOR 0x29d
#define BLUES_MINOR 0x4e9
#define BLUES_COMBINED 0x6fd
#define LYDIAN_5 0xb55
#define JAZZ_MINOR 0xaad
#define HALF_DIMINISHED 0x56d
#define AUGMENTED 0x999

#define HUNGARIAN_MAJOR 0x9cd
#define HUNGARIAN_MINOR 0x6d9
#define NEAPOLITAN 0x9ab
#define SPANISH 0x5bb
#define GREEK 0x59d
#define JEWISH1 0x2db
#define JEWISH2 0x6cd
#define INDIAN1 0x9cd
#define INDIAN2 0xacd
#define INDIAN3 0x3a7
#define INDIAN4 0xcb9
#define MIDEAST1 0x9b3
#define MIDEAST2 0x973
#define MIDEAST3 0x66b
#define MIDEAST4 0x673

#define PENTATONIC_I 0x295
#define PENTATONIC_II 0x4a5
#define PENTATONIC_III 0x529
#define PENTATONIC_IV 0x2a5
#define PENTATONIC_V 0x4a9
#define HIRAJOSHI 0x18d
#define INSEN 0x4a3
#define KOKIN_JOSHI 0x1a5
#define AKEBONO 0x28d
#define RYUKUAN 0x8b1
#define ABHOGI 0x22d
#define BHUPKALI 0x195
#define HINDOLAM 0x529
#define BHUPALAM 0x18b
#define AMRITAVARSHINI 0x8d1

#define OCTATONIC 0xb6d
#define ACOUSTIC 0x6d5
//#define AUGMENTED
#define TRITONE 0x4d3
#define LEADING_WHOLE_TONE 0xd55
#define ENIGMATIC 0xd53
#define SCRIABIN 0x655
#define TCHEREPNIN 0xbbb
#define MESSIAEN_I 0x555
#define MESSIANE_II 0x6db
#define MESSIANE_III 0xddd
#define MESSIANE_IV 0x9e7
#define MESSIANE_V 0x8e3
#define MESSIANE_VI 0xd75
#define MESSIANE_VII 0xbef

#define NATURAL_MAJOR 0xab5
//#define LYDIAN
//#define MIXOLYDIAN
#define MAJOR_MINOR 0x5b5
#define HARMONIC_MAJOR 0x9b5
#define DOUBLE_HARMONIC_MAJOR 0x9b3
#define NEAPOLITAN_MAJOR 0xab3
#define BEBOP_MAJOR 0xbb5
#define HEXATONIC_1_MAJOR 0xa95
#define HEXATONIC_2_MAJOR 0x2b5
#define PENTATONIC_1_MAJOR 0x295
#define PENTATONIC_2_MAJOR 0x8b1
#define PENTATONIC_3_MAJOR 0xa91

#define NATURAL_MINOR 0x5ad
//#define DORIAN
//#define PHRYGIAN
#define MINOR_MAJOR 0xaad
//#define HARMONIC_MINOR
#define DOUBLE_HARMONIC_MINOR 0x9cd
#define NEAPOLITAIN_MINOR 0x9ab
#define MINOR_LOCRIAN 0x56d
//#define BLUES_MINOR
#define BEBOP_MINOR 0xdad
#define HEXATONIC_1_MINOR 0x58d
#define HEXATONIC_2_MINOR 0x1ad
#define PENTATONIC_1_MINOR 0x18d
#define PENTATONIC_2_MINOR 0x4a9
#define PENTATONIC_3_MINOR 0x589

const PROGMEM uint16_t scales[NUM_SCALES] = {
  CHROMATIC, MAJOR, MINOR, HARMONIC_MINOR, MAJOR_PENTATONIC,
  MINOR_PENTATONIC, BLUES, JAPANESE, FREYGISH, ROMANI,
  ARABIC, ALTERED, WHOLE_TONE, HW_DIMINISHED, WH_DIMINISHED,

  IONIAN, DORIAN, PHRYGIAN, LYDIAN, MIXOLYDIAN,
  AEOLIAN, LOCRIAN, IONIAN_B2, DORIAN_B5, HARMONIC_PHRYGIAN,
  PHRYGIAN_MAJOR, LYDIAN_B3, MAJOR_LOCRIAN, MINOR_LOCRIAN, SUPER_LOCRIAN,

  LYDIAN_7B, ALTERED, DIMINISHED, MIXOLYDIAN_B13, MIXOLYDIAN_B9_B13,
  LYDIAN_7B_B2, BEBOP, WHOLE_TONE, BLUES_MAJOR, BLUES_MINOR,
  BLUES_COMBINED, LYDIAN_5, JAZZ_MINOR, HALF_DIMINISHED, AUGMENTED,

  HUNGARIAN_MAJOR, HUNGARIAN_MINOR, NEAPOLITAN, SPANISH, GREEK,
  JEWISH1, JEWISH2, INDIAN1, INDIAN2, INDIAN3,
  INDIAN4, MIDEAST1, MIDEAST2, MIDEAST3, MIDEAST4,

  PENTATONIC_I, PENTATONIC_II, PENTATONIC_III, PENTATONIC_IV, PENTATONIC_V,
  HIRAJOSHI, INSEN, KOKIN_JOSHI, AKEBONO, RYUKUAN,
  ABHOGI, BHUPKALI, HINDOLAM, BHUPALAM, AMRITAVARSHINI,

  OCTATONIC, ACOUSTIC, AUGMENTED, TRITONE, LEADING_WHOLE_TONE,
  ENIGMATIC, SCRIABIN, TCHEREPNIN, MESSIAEN_I, MESSIANE_II,
  MESSIANE_III, MESSIANE_IV, MESSIANE_V, MESSIANE_VI, MESSIANE_VII,

  NATURAL_MAJOR, LYDIAN, MIXOLYDIAN, MAJOR_MINOR, HARMONIC_MAJOR,
  DOUBLE_HARMONIC_MAJOR, NEAPOLITAN_MAJOR, MAJOR_LOCRIAN, BLUES_MAJOR, BEBOP_MAJOR,
  HEXATONIC_1_MAJOR, HEXATONIC_2_MAJOR, PENTATONIC_1_MAJOR, PENTATONIC_2_MAJOR, PENTATONIC_3_MAJOR,

  NATURAL_MINOR, DORIAN, PHRYGIAN, MINOR_MAJOR, HARMONIC_MINOR,
  DOUBLE_HARMONIC_MINOR, NEAPOLITAIN_MINOR, MINOR_LOCRIAN, BLUES_MINOR, BEBOP_MINOR,
  HEXATONIC_1_MINOR, HEXATONIC_2_MINOR, PENTATONIC_1_MINOR, PENTATONIC_2_MINOR, PENTATONIC_3_MINOR
};

const char Chromatic_string[] PROGMEM = "Chromatic";
const char Major_string[] PROGMEM = "Major";
const char Minor_string[] PROGMEM = "Minor";
const char Harm_Min_string[] PROGMEM = "Harm Min";
const char Maj_Penta_string[] PROGMEM = "Maj Penta";
const char Min_Penta_string[] PROGMEM = "Min Penta";
const char Blues_string[] PROGMEM = "Blues";
const char Japanese_string[] PROGMEM = "Japanese";
const char Freygish_string[] PROGMEM = "Freygish";
const char Romani_string[] PROGMEM = "Romani";
const char Arabic_string[] PROGMEM = "Arabic";
const char Altered_string[] PROGMEM = "Altered";
const char Whl_Tone_string[] PROGMEM = "Whl Tone";
const char H_W_Dim_string[] PROGMEM = "H-W Dim";
const char W_H_Dim_string[] PROGMEM = "W-H Dim";

const char Ionian_string[] PROGMEM = "Ionian";
const char Dorian_string[] PROGMEM = "Dorian";
const char Phrygian_string[] PROGMEM = "Phrygian";
const char Lydian_string[] PROGMEM = "Lydian";
const char Mixolydian_string[] PROGMEM = "Mixolydian";
const char Aeolian_string[] PROGMEM = "Aeolian";
const char Locrian_string[] PROGMEM = "Locrian";
const char Ionian_b2_string[] PROGMEM = "Ionian b2";
const char Dorian_b5_string[] PROGMEM = "Dorian b5";
const char Harm_Phry_string[] PROGMEM = "Harm Phry";
const char Phry_Maj_string[] PROGMEM = "Phry Maj";
const char Lydian_b3_string[] PROGMEM = "Lydian b3";
const char Maj_Locr_string[] PROGMEM = "Maj Locr";
const char Min_Locr_string[] PROGMEM = "Min Locr";
const char Super_Locr_string[] PROGMEM = "Super Locr";

const char Lydian_7b_string[] PROGMEM = "Lydian 7b";
//const char Altered_string[] PROGMEM = "Altered";
const char Diminished_string[] PROGMEM = "Diminished";
const char Mix_b13_string[] PROGMEM = "Mix b13";
const char Mix_b9_b13_string[] PROGMEM = "Mix b9 b13";
const char Lyd_7b_b2_string[] PROGMEM = "Lyd 7b b2";
const char Bebop_string[] PROGMEM = "Bebop";
//const char Whl_Tone_string[] PROGMEM = "Whl Tone";
const char Blues_Maj_string[] PROGMEM = "Blues Maj";
const char Blues_Min_string[] PROGMEM = "Blues Min";
const char Blues_Comb_string[] PROGMEM = "Blues Comb";
const char Lydian_5_string[] PROGMEM = "Lydian #5";
const char Jazz_Min_string[] PROGMEM = "Jazz Min";
const char Half_Dim_string[] PROGMEM = "Half Dim";
const char Augmented_string[] PROGMEM = "Augmented";

const char Hungar_Min_string[] PROGMEM = "Hungar Min";
const char Hungar_Maj_string[] PROGMEM = "Hungar Maj";
const char Neapolitan_string[] PROGMEM = "Neapolitan";
const char Spanish_string[] PROGMEM = "Spanish";
const char Greek_string[] PROGMEM = "Greek";
const char Jewish_1_string[] PROGMEM = "Jewish 1";
const char Jewish_2_string[] PROGMEM = "Jewish 2";
const char Indian_1_string[] PROGMEM = "Indian 1";
const char Indian_2_string[] PROGMEM = "Indian 2";
const char Indian_3_string[] PROGMEM = "Indian 3";
const char Indian_4_string[] PROGMEM = "Indian 4";
const char Mid_East_1_string[] PROGMEM = "Mid East 1";
const char Mid_East_2_string[] PROGMEM = "Mid East 2";
const char Mid_East_3_string[] PROGMEM = "Mid East 3";
const char Mid_East_4_string[] PROGMEM = "Mid East 4";

const char Pent_I_string[] PROGMEM = "Penta   I";
const char Pent_II_string[] PROGMEM = "Penta  II";
const char Pent_III_string[] PROGMEM = "Penta III";
const char Pent_IV_string[] PROGMEM = "Penta  IV";
const char Pent_V_string[] PROGMEM = "Penta   V";
const char Hirajoshi_string[] PROGMEM = "Hirajoshi";
const char Insen_string[] PROGMEM = "Insen";
const char KokinJoshi_string[] PROGMEM = "KokinJoshi";
const char Akebono_string[] PROGMEM = "Akebono";
const char Ryukuan_string[] PROGMEM = "Ryukuan";
const char Abhogi_string[] PROGMEM = "Abhogi";
const char Bhupkali_string[] PROGMEM = "Bhupkali";
const char Hindolam_string[] PROGMEM = "Hindolam";
const char Bhupalam_string[] PROGMEM = "Bhupalam";
const char Amritavars_string[] PROGMEM = "Amritavars";

const char Octatonic_string[] PROGMEM = "Octatonic";
const char Acoustic_string[] PROGMEM = "Acoustic";
//const char Augmented_string[] PROGMEM = "Augmented";
const char Tritone_string[] PROGMEM = "Tritone";
const char LeadingWhl_string[] PROGMEM = "LeadingWhl";
const char Enigmatic_string[] PROGMEM = "Enigmatic";
const char Scriabin_string[] PROGMEM = "Scriabin";
const char Tcherepnin_string[] PROGMEM = "Tcherepnin";
const char Messiaen_1_string[] PROGMEM = "Messiaen 1";
const char Messiaen_2_string[] PROGMEM = "Messiaen 2";
const char Messiaen_3_string[] PROGMEM = "Messiaen 3";
const char Messiaen_4_string[] PROGMEM = "Messiaen 4";
const char Messiaen_5_string[] PROGMEM = "Messiaen 5";
const char Messiaen_6_string[] PROGMEM = "Messiaen 6";
const char Messiaen_7_string[] PROGMEM = "Messiaen 7";

const char Nat_Maj_string[] PROGMEM = "Natur Maj";
//const char Lydian_string[] PROGMEM = "Lydian";
//const char Mixolydian_string[] PROGMEM = "Mixolydian";
const char Maj_Min_string[] PROGMEM = "Maj Min";
const char Harm_Maj_string[] PROGMEM = "Harm Maj";
const char DblHarmMaj_string[] PROGMEM = "DblHarmMaj";
const char NaepolMaj_string[] PROGMEM = "Naepol Maj";
const char Major_Loc_string[] PROGMEM = "Major Loc";
const char BluesMaj_string[] PROGMEM = "Blues Maj";
const char Bebop_Maj_string[] PROGMEM = "Bebop Maj";
const char Hexa_1_Maj_string[] PROGMEM = "Hexa 1 Maj";
const char Hexa_2_Maj_string[] PROGMEM = "Hexa 2 Maj";
const char Penta1_Maj_string[] PROGMEM = "Penta1 Maj";
const char Penta2_Maj_string[] PROGMEM = "Penta2 Maj";
const char Penta3_Maj_string[] PROGMEM = "Penta3 Maj";

const char Nat_Min_string[] PROGMEM = "Natur Min";
//const char Dorian_string[] PROGMEM = "Dorian";
//const char Phrygian_string[] PROGMEM = "Phrygian";
const char Min_Maj_string[] PROGMEM = "Min Maj";
//const char Harm_Min_string[] PROGMEM = "Harm Min";
const char DblHarmMin_string[] PROGMEM = "DblHarmMin";
const char NeapolMin_string[] PROGMEM = "Neapol Min";
const char Minor_Loc_string[] PROGMEM = "Minor Loc";
//const char Blues_Min_string[] PROGMEM = "Blues Min";
const char Bebop_Min_string[] PROGMEM = "Bebop Min";
const char Hexa_1_Min_string[] PROGMEM = "Hexa 1 Min";
const char Hexa_2_Min_string[] PROGMEM = "Hexa 2 Min";
const char Penta1_Min_string[] PROGMEM = "Penta1 Min";
const char Penta2_Min_string[] PROGMEM = "Penta2 Min";
const char Penta3_Min_string[] PROGMEM = "Penta3 Min";

const char *const short_scale_names[NUM_SCALES] PROGMEM = {
  Chromatic_string, Major_string, Minor_string, Harm_Min_string, Maj_Penta_string,
  Min_Penta_string, Blues_string, Japanese_string, Freygish_string, Romani_string,
  Arabic_string, Altered_string, Whl_Tone_string, H_W_Dim_string, W_H_Dim_string,

  Ionian_string, Dorian_string, Phrygian_string, Lydian_string, Mixolydian_string,
  Aeolian_string, Locrian_string, Ionian_b2_string, Dorian_b5_string, Harm_Phry_string,
  Phry_Maj_string, Lydian_b3_string, Maj_Locr_string, Min_Locr_string, Super_Locr_string,

  Lydian_7b_string, Altered_string, Diminished_string, Mix_b13_string, Mix_b9_b13_string,
  Lyd_7b_b2_string, Bebop_string, Whl_Tone_string, Blues_Maj_string, Blues_Min_string,
  Blues_Comb_string, Lydian_5_string, Jazz_Min_string, Half_Dim_string, Augmented_string,

  Hungar_Min_string, Hungar_Maj_string, Neapolitan_string, Spanish_string, Greek_string,
  Jewish_1_string, Jewish_2_string, Indian_1_string, Indian_2_string, Indian_3_string,
  Indian_4_string, Mid_East_1_string, Mid_East_2_string, Mid_East_3_string, Mid_East_4_string,

  Pent_I_string, Pent_II_string, Pent_III_string, Pent_IV_string, Pent_V_string,
  Hirajoshi_string, Insen_string, KokinJoshi_string, Akebono_string, Ryukuan_string,
  Abhogi_string, Bhupkali_string, Hindolam_string, Bhupalam_string, Amritavars_string,

  Octatonic_string, Acoustic_string, Augmented_string, Tritone_string, LeadingWhl_string,
  Enigmatic_string, Scriabin_string, Tcherepnin_string, Messiaen_1_string, Messiaen_2_string,
  Messiaen_3_string, Messiaen_4_string, Messiaen_5_string, Messiaen_6_string, Messiaen_7_string,

  Nat_Maj_string, Lydian_string, Mixolydian_string, Maj_Min_string, Harm_Maj_string,
  DblHarmMaj_string, NaepolMaj_string, Major_Loc_string, BluesMaj_string, Bebop_Maj_string,
  Hexa_1_Maj_string, Hexa_2_Maj_string, Penta1_Maj_string, Penta2_Maj_string, Penta3_Maj_string,

  Nat_Min_string, Dorian_string, Phrygian_string, Min_Maj_string, Harm_Min_string,
  DblHarmMin_string, NeapolMin_string, Minor_Loc_string, Blues_Min_string, Bebop_Min_string,
  Hexa_1_Min_string, Hexa_2_Min_string, Penta1_Min_string, Penta2_Min_string, Penta3_Min_string
};


const char key_C[2] PROGMEM = "C";
const char key_Csharp[6] PROGMEM = "C#/Db";
const char key_D[2] PROGMEM = "D";
const char key_Dsharp[6] PROGMEM = "D#/Eb";
const char key_E[2] PROGMEM = "E";
const char key_F[2] PROGMEM = "F";
const char key_Fsharp[6] PROGMEM = "F#/Gb";
const char key_G[2] PROGMEM = "G";
const char key_Gsharp[6] PROGMEM = "G#/Ab";
const char key_A[2] PROGMEM = "A";
const char key_Asharp[6] PROGMEM = "A#/Bb";
const char key_B[2] PROGMEM = "B";

const char *const notes[12] PROGMEM = {
  key_C, key_Csharp, key_D, key_Dsharp, key_E, key_F,
  key_Fsharp, key_G, key_Gsharp, key_A, key_Asharp, key_B
};


// Draws the layout smaller or bigger
#define PROP 2

#define ROOT_DOT_SIZE 3





// [0] = 0 -> white key
//     = 1 -> black key
// [1] delta x to next note (white->black or black->white might not be the same
//                           if one is thinner/wider than the other)
// [2] heigth of lower left corder
// [3] width
// [4] height
// [5] rounded corner radius
const int8_t piano_layout[12][6] =
{
  {0, 5, 2, 7, 14, 2},    // W
  {1, 3, 0, 5, 12, 2},    // K
  {0, 5, 2, 7, 14, 2},    // W
  {1, 3, 0, 5, 12, 2},   // K
  {0, 8, 2, 7, 14, 2},   // W
  {0, 5, 2, 7, 14, 2},   // W
  {1, 3, 0, 5, 12, 2},   // K
  {0, 5, 2, 7, 14, 2},   // W
  {1, 3, 0, 5, 12, 2},   // K
  {0, 5, 2, 7, 14, 2},   // W
  {1, 3, 0, 5, 12, 2},   // K
  {0, 8, 2, 7, 14, 2}    // W
};


const int8_t beatstep_layout[12][6] =
{
  {0, 0, 9, 7, 7, 2},    // W
  {1, 8, 1, 7, 7, 2},    // K
  {0, 0, 9, 7, 7, 2},    // W
  {1, 8, 1, 7, 7, 2},   // K
  {0, 8, 9, 7, 7, 2},   // W
  {0, 0, 9, 7, 7, 2},   // W
  {1, 8, 1, 7, 7, 2},   // K
  {0, 0, 9, 7, 7, 2},   // W
  {1, 8, 1, 7, 7, 2},   // K
  {0, 0, 9, 7, 7, 2},   // W
  {1, 8, 1, 7, 7, 2},   // K
  {0, 8, 9, 7, 7, 2}    // W

};

const int8_t (*keyboard_layouts[2])[6] = {piano_layout, beatstep_layout};
const int8_t (*keyboard_layout)[6];
