// /*
//     Volcanos Library. Written by Benjamin Jack Cullen.

//     This library has been designed for low memory requirements.
// */

// #include "volcanos.h"
// #include <string.h>
// #include <math.h>

// // ----------------------------------------------------------------------------------------
// /*
//    Populated by : getVolcanoName via volcano_index_key.
// */
// // ----------------------------------------------------------------------------------------
// static char volcano_name_buffer[MAX_GLOBAL_ELEMENT_SIZE];

// // ----------------------------------------------------------------------------------------
// /*
//    Accessed by : volcano_index_key.
//    Designed to require very low memory rather than storing all names in an array.
// */
// // ----------------------------------------------------------------------------------------
// const char* getVolcanoName(int index) {
//     if (index < 0 || index >= MAX_VOLCANOS) {
//         strcpy(volcano_name_buffer, "undefined");
//         return volcano_name_buffer;
//     }
//     if      (index == 0) {strcpy(volcano_name_buffer, "Abu");}
//     else if (index == 1) {strcpy(volcano_name_buffer, "Acamarachi");}
//     else if (index == 2) {strcpy(volcano_name_buffer, "Acatenango");}
//     else if (index == 3) {strcpy(volcano_name_buffer, "Acigöl-Nevsehir");}
//     else if (index == 4) {strcpy(volcano_name_buffer, "Adams");}
//     else if (index == 5) {strcpy(volcano_name_buffer, "Adams Seamount");}
//     else if (index == 6) {strcpy(volcano_name_buffer, "Adatara");}
//     else if (index == 7) {strcpy(volcano_name_buffer, "Adwa");}
//     else if (index == 8) {strcpy(volcano_name_buffer, "Afderà");}
//     else if (index == 9) {strcpy(volcano_name_buffer, "Agrigan");}
//     else if (index == 10) {strcpy(volcano_name_buffer, "Agua");}
//     else if (index == 11) {strcpy(volcano_name_buffer, "Agua de Pau");}
//     else if (index == 12) {strcpy(volcano_name_buffer, "Aguilera");}
//     else if (index == 13) {strcpy(volcano_name_buffer, "Agung");}
//     else if (index == 14) {strcpy(volcano_name_buffer, "Ahyi");}
//     else if (index == 15) {strcpy(volcano_name_buffer, "Akademia Nauk");}
//     else if (index == 16) {strcpy(volcano_name_buffer, "Akagi");}
//     else if (index == 17) {strcpy(volcano_name_buffer, "Akan");}
//     else if (index == 18) {strcpy(volcano_name_buffer, "Akhtang");}
//     else if (index == 19) {strcpy(volcano_name_buffer, "Akita Komagatake");}
//     else if (index == 20) {strcpy(volcano_name_buffer, "Akita Yakeyama");}
//     else if (index == 21) {strcpy(volcano_name_buffer, "Akuseki-jima");}
//     else if (index == 22) {strcpy(volcano_name_buffer, "Akutan");}
//     else if (index == 23) {strcpy(volcano_name_buffer, "Alaid");}
//     else if (index == 24) {strcpy(volcano_name_buffer, "Alamagan");}
//     else if (index == 25) {strcpy(volcano_name_buffer, "Alayta");}
//     else if (index == 26) {strcpy(volcano_name_buffer, "Alban Hills");}
//     else if (index == 27) {strcpy(volcano_name_buffer, "Alcedo");}
//     else if (index == 28) {strcpy(volcano_name_buffer, "Ale Bagu");}
//     else if (index == 29) {strcpy(volcano_name_buffer, "Alid");}
//     else if (index == 30) {strcpy(volcano_name_buffer, "Ambrym");}
//     else if (index == 31) {strcpy(volcano_name_buffer, "Ambrym");}
//     else if (index == 32) {strcpy(volcano_name_buffer, "Anatahan");}
//     else if (index == 33) {strcpy(volcano_name_buffer, "Aneityum");}
//     else if (index == 34) {strcpy(volcano_name_buffer, "Aniakchak");}
//     else if (index == 35) {strcpy(volcano_name_buffer, "Antisana");}
//     else if (index == 36) {strcpy(volcano_name_buffer, "Aoba");}
//     else if (index == 37) {strcpy(volcano_name_buffer, "Ararat");}
//     else if (index == 38) {strcpy(volcano_name_buffer, "Arenal");}
//     else if (index == 39) {strcpy(volcano_name_buffer, "Arintica");}
//     else if (index == 40) {strcpy(volcano_name_buffer, "Asacha");}
//     else if (index == 41) {strcpy(volcano_name_buffer, "Asama");}
//     else if (index == 42) {strcpy(volcano_name_buffer, "Ascensión");}
//     else if (index == 43) {strcpy(volcano_name_buffer, "Askja");}
//     else if (index == 44) {strcpy(volcano_name_buffer, "Aso");}
//     else if (index == 45) {strcpy(volcano_name_buffer, "Atitlán");}
//     else if (index == 46) {strcpy(volcano_name_buffer, "Atka");}
//     else if (index == 47) {strcpy(volcano_name_buffer, "Atsonupuri");}
//     else if (index == 48) {strcpy(volcano_name_buffer, "Augustine");}
//     else if (index == 49) {strcpy(volcano_name_buffer, "Avachinsky");}
//     else if (index == 50) {strcpy(volcano_name_buffer, "Awu");}
//     else if (index == 51) {strcpy(volcano_name_buffer, "Axial Seamount");}
//     else if (index == 52) {strcpy(volcano_name_buffer, "Ayelu");}
//     else if (index == 53) {strcpy(volcano_name_buffer, "Azufre");}
//     else if (index == 54) {strcpy(volcano_name_buffer, "Azuma");}
//     else if (index == 55) {strcpy(volcano_name_buffer, "Babuyon Claro");}
//     else if (index == 56) {strcpy(volcano_name_buffer, "Bachelor");}
//     else if (index == 57) {strcpy(volcano_name_buffer, "Bagana");}
//     else if (index == 58) {strcpy(volcano_name_buffer, "Baker");}
//     else if (index == 59) {strcpy(volcano_name_buffer, "Bam");}
//     else if (index == 60) {strcpy(volcano_name_buffer, "Bamus");}
//     else if (index == 61) {strcpy(volcano_name_buffer, "Bandai");}
//     else if (index == 62) {strcpy(volcano_name_buffer, "Barren Island");}
//     else if (index == 63) {strcpy(volcano_name_buffer, "Barrier");}
//     else if (index == 64) {strcpy(volcano_name_buffer, "Bazman");}
//     else if (index == 65) {strcpy(volcano_name_buffer, "Beerenberg");}
//     else if (index == 66) {strcpy(volcano_name_buffer, "Belknap");}
//     else if (index == 67) {strcpy(volcano_name_buffer, "Bezymianny");}
//     else if (index == 68) {strcpy(volcano_name_buffer, "Biliran");}
//     else if (index == 69) {strcpy(volcano_name_buffer, "Bolshoi Semiachik");}
//     else if (index == 70) {strcpy(volcano_name_buffer, "Bulusan");}
//     else if (index == 71) {strcpy(volcano_name_buffer, "Butajira");}
//     else if (index == 72) {strcpy(volcano_name_buffer, "Butajira");}
//     else if (index == 73) {strcpy(volcano_name_buffer, "Cameroon (Mount)");}
//     else if (index == 74) {strcpy(volcano_name_buffer, "Camiguin de Babuyanes");}
//     else if (index == 75) {strcpy(volcano_name_buffer, "Canlaon");}
//     else if (index == 76) {strcpy(volcano_name_buffer, "Capulin");}
//     else if (index == 77) {strcpy(volcano_name_buffer, "Carrizozo");}
//     else if (index == 78) {strcpy(volcano_name_buffer, "Cayambe");}
//     else if (index == 79) {strcpy(volcano_name_buffer, "Cereme");}
//     else if (index == 80) {strcpy(volcano_name_buffer, "Chaine des Puys");}
//     else if (index == 81) {strcpy(volcano_name_buffer, "Chiginagak");}
//     else if (index == 82) {strcpy(volcano_name_buffer, "Chikurachki");}
//     else if (index == 83) {strcpy(volcano_name_buffer, "Chimborazo");}
//     else if (index == 84) {strcpy(volcano_name_buffer, "Cleveland");}
//     else if (index == 85) {strcpy(volcano_name_buffer, "Coatepeque");}
//     else if (index == 86) {strcpy(volcano_name_buffer, "Colima");}
//     else if (index == 87) {strcpy(volcano_name_buffer, "Columbia River Flood Basalts");}
//     else if (index == 88) {strcpy(volcano_name_buffer, "Concepción");}
//     else if (index == 89) {strcpy(volcano_name_buffer, "Copahue");}
//     else if (index == 90) {strcpy(volcano_name_buffer, "Cosigüina");}
//     else if (index == 91) {strcpy(volcano_name_buffer, "Cotopaxi");}
//     else if (index == 92) {strcpy(volcano_name_buffer, "Crater Lake");}
//     else if (index == 93) {strcpy(volcano_name_buffer, "Craters of the Moon");}
//     else if (index == 94) {strcpy(volcano_name_buffer, "Daisetsu");}
//     else if (index == 95) {strcpy(volcano_name_buffer, "Dakataua");}
//     else if (index == 96) {strcpy(volcano_name_buffer, "Damavand");}
//     else if (index == 97) {strcpy(volcano_name_buffer, "Deccan Traps");}
//     else if (index == 98) {strcpy(volcano_name_buffer, "Dempo");}
//     else if (index == 99) {strcpy(volcano_name_buffer, "Devils Garden");}
//     else if (index == 100) {strcpy(volcano_name_buffer, "Devils Tower");}
//     else if (index == 101) {strcpy(volcano_name_buffer, "Diamond Craters");}
//     else if (index == 102) {strcpy(volcano_name_buffer, "Dieng Volcanic Complex");}
//     else if (index == 103) {strcpy(volcano_name_buffer, "Diky Greben");}
//     else if (index == 104) {strcpy(volcano_name_buffer, "Dubbi");}
//     else if (index == 105) {strcpy(volcano_name_buffer, "Dukono");}
//     else if (index == 106) {strcpy(volcano_name_buffer, "East Epi");}
//     else if (index == 107) {strcpy(volcano_name_buffer, "Ebeko");}
//     else if (index == 108) {strcpy(volcano_name_buffer, "Egmont (Taranaki)");}
//     else if (index == 109) {strcpy(volcano_name_buffer, "Egon");}
//     else if (index == 110) {strcpy(volcano_name_buffer, "El Chichon");}
//     else if (index == 111) {strcpy(volcano_name_buffer, "Elbrus");}
//     else if (index == 112) {strcpy(volcano_name_buffer, "Erebus");}
//     else if (index == 113) {strcpy(volcano_name_buffer, "Erta Ale");}
//     else if (index == 114) {strcpy(volcano_name_buffer, "Etna");}
//     else if (index == 115) {strcpy(volcano_name_buffer, "Fantale");}
//     else if (index == 116) {strcpy(volcano_name_buffer, "Fernandina");}
//     else if (index == 117) {strcpy(volcano_name_buffer, "Fisher");}
//     else if (index == 118) {strcpy(volcano_name_buffer, "Fogo");}
//     else if (index == 119) {strcpy(volcano_name_buffer, "Fort Rock");}
//     else if (index == 120) {strcpy(volcano_name_buffer, "Frosty Peak");}
//     else if (index == 121) {strcpy(volcano_name_buffer, "Fuego");}
//     else if (index == 122) {strcpy(volcano_name_buffer, "Fuerteventura");}
//     else if (index == 123) {strcpy(volcano_name_buffer, "Fuji");}
//     else if (index == 124) {strcpy(volcano_name_buffer, "Furnas");}
//     else if (index == 125) {strcpy(volcano_name_buffer, "Fuss Peak");}
//     else if (index == 126) {strcpy(volcano_name_buffer, "Galeras");}
//     else if (index == 127) {strcpy(volcano_name_buffer, "Galunggung");}
//     else if (index == 128) {strcpy(volcano_name_buffer, "Gamalama");}
//     else if (index == 129) {strcpy(volcano_name_buffer, "Gamchen");}
//     else if (index == 130) {strcpy(volcano_name_buffer, "Gamkonora");}
//     else if (index == 131) {strcpy(volcano_name_buffer, "Gareloi");}
//     else if (index == 132) {strcpy(volcano_name_buffer, "Gaua");}
//     else if (index == 133) {strcpy(volcano_name_buffer, "Gede");}
//     else if (index == 134) {strcpy(volcano_name_buffer, "Glacier Peak");}
//     else if (index == 135) {strcpy(volcano_name_buffer, "Gorely");}
//     else if (index == 136) {strcpy(volcano_name_buffer, "Graciosa");}
//     else if (index == 137) {strcpy(volcano_name_buffer, "Gran Canaria");}
//     else if (index == 138) {strcpy(volcano_name_buffer, "Great Sitkin");}
//     else if (index == 139) {strcpy(volcano_name_buffer, "Griggs");}
//     else if (index == 140) {strcpy(volcano_name_buffer, "Grímsvötn");}
//     else if (index == 141) {strcpy(volcano_name_buffer, "Guagua Pichincha");}
//     else if (index == 142) {strcpy(volcano_name_buffer, "Guallatiri");}
//     else if (index == 143) {strcpy(volcano_name_buffer, "Guguan");}
//     else if (index == 144) {strcpy(volcano_name_buffer, "Hakone");}
//     else if (index == 145) {strcpy(volcano_name_buffer, "Heard");}
//     else if (index == 146) {strcpy(volcano_name_buffer, "Hibok-hibok");}
//     else if (index == 147) {strcpy(volcano_name_buffer, "Hierro");}
//     else if (index == 148) {strcpy(volcano_name_buffer, "Hood");}
//     else if (index == 149) {strcpy(volcano_name_buffer, "Hualalai");}
//     else if (index == 150) {strcpy(volcano_name_buffer, "Ibu");}
//     else if (index == 151) {strcpy(volcano_name_buffer, "Ijen");}
//     else if (index == 152) {strcpy(volcano_name_buffer, "Iliamna");}
//     else if (index == 153) {strcpy(volcano_name_buffer, "Iliboleng");}
//     else if (index == 154) {strcpy(volcano_name_buffer, "Iliniza");}
//     else if (index == 155) {strcpy(volcano_name_buffer, "Ilopango");}
//     else if (index == 156) {strcpy(volcano_name_buffer, "Ilyinsky");}
//     else if (index == 157) {strcpy(volcano_name_buffer, "Ioto (Iwo-jima)");}
//     else if (index == 158) {strcpy(volcano_name_buffer, "Iraya");}
//     else if (index == 159) {strcpy(volcano_name_buffer, "Irazu");}
//     else if (index == 160) {strcpy(volcano_name_buffer, "Isanotski");}
//     else if (index == 161) {strcpy(volcano_name_buffer, "Izalco");}
//     else if (index == 162) {strcpy(volcano_name_buffer, "Iztaccihuatl");}
//     else if (index == 163) {strcpy(volcano_name_buffer, "Izu-Tobu");}
//     else if (index == 164) {strcpy(volcano_name_buffer, "Jailolo");}
//     else if (index == 165) {strcpy(volcano_name_buffer, "Jefferson");}
//     else if (index == 166) {strcpy(volcano_name_buffer, "Jordan Craters");}
//     else if (index == 167) {strcpy(volcano_name_buffer, "Kadovar");}
//     else if (index == 168) {strcpy(volcano_name_buffer, "Kaguyak");}
//     else if (index == 169) {strcpy(volcano_name_buffer, "Kahoolawe");}
//     else if (index == 170) {strcpy(volcano_name_buffer, "Kambalny");}
//     else if (index == 171) {strcpy(volcano_name_buffer, "Kanaga");}
//     else if (index == 172) {strcpy(volcano_name_buffer, "Karangetang [Api Siau]");}
//     else if (index == 173) {strcpy(volcano_name_buffer, "Karisimbi");}
//     else if (index == 174) {strcpy(volcano_name_buffer, "Karisimbi");}
//     else if (index == 175) {strcpy(volcano_name_buffer, "Karthala");}
//     else if (index == 176) {strcpy(volcano_name_buffer, "Karymsky");}
//     else if (index == 177) {strcpy(volcano_name_buffer, "Katla");}
//     else if (index == 178) {strcpy(volcano_name_buffer, "Katmai");}
//     else if (index == 179) {strcpy(volcano_name_buffer, "Katwe-Kikorongo");}
//     else if (index == 180) {strcpy(volcano_name_buffer, "Kavachi");}
//     else if (index == 181) {strcpy(volcano_name_buffer, "Kell");}
//     else if (index == 182) {strcpy(volcano_name_buffer, "Kelut");}
//     else if (index == 183) {strcpy(volcano_name_buffer, "Kerinci");}
//     else if (index == 184) {strcpy(volcano_name_buffer, "Khodutka");}
//     else if (index == 185) {strcpy(volcano_name_buffer, "Kialagvik");}
//     else if (index == 186) {strcpy(volcano_name_buffer, "Kick 'em Jenny");}
//     else if (index == 187) {strcpy(volcano_name_buffer, "Kikai");}
//     else if (index == 188) {strcpy(volcano_name_buffer, "Kikhpinych");}
//     else if (index == 189) {strcpy(volcano_name_buffer, "Kilauea");}
//     else if (index == 190) {strcpy(volcano_name_buffer, "Kilimanjaro");}
//     else if (index == 191) {strcpy(volcano_name_buffer, "Kirishima");}
//     else if (index == 192) {strcpy(volcano_name_buffer, "Kliuchevskoi");}
//     else if (index == 193) {strcpy(volcano_name_buffer, "Komagatake");}
//     else if (index == 194) {strcpy(volcano_name_buffer, "Kone");}
//     else if (index == 195) {strcpy(volcano_name_buffer, "Koryaksky");}
//     else if (index == 196) {strcpy(volcano_name_buffer, "Koshelev");}
//     else if (index == 197) {strcpy(volcano_name_buffer, "Krafla");}
//     else if (index == 198) {strcpy(volcano_name_buffer, "Krakatau");}
//     else if (index == 199) {strcpy(volcano_name_buffer, "Krasheninnikov");}
//     else if (index == 200) {strcpy(volcano_name_buffer, "Kronotsky");}
//     else if (index == 201) {strcpy(volcano_name_buffer, "Ksudach");}
//     else if (index == 202) {strcpy(volcano_name_buffer, "Kukak");}
//     else if (index == 203) {strcpy(volcano_name_buffer, "Kusatsu-Shirane");}
//     else if (index == 204) {strcpy(volcano_name_buffer, "Kutcharo");}
//     else if (index == 205) {strcpy(volcano_name_buffer, "Kuttara");}
//     else if (index == 206) {strcpy(volcano_name_buffer, "Kuwae");}
//     else if (index == 207) {strcpy(volcano_name_buffer, "La Palma");}
//     else if (index == 208) {strcpy(volcano_name_buffer, "Lamington");}
//     else if (index == 209) {strcpy(volcano_name_buffer, "Langila");}
//     else if (index == 210) {strcpy(volcano_name_buffer, "Lanzarote");}
//     else if (index == 211) {strcpy(volcano_name_buffer, "Láscar");}
//     else if (index == 212) {strcpy(volcano_name_buffer, "Lereboleng");}
//     else if (index == 213) {strcpy(volcano_name_buffer, "Lewotobi");}
//     else if (index == 214) {strcpy(volcano_name_buffer, "Lewotolo");}
//     else if (index == 215) {strcpy(volcano_name_buffer, "Liamuiga");}
//     else if (index == 216) {strcpy(volcano_name_buffer, "Lipari");}
//     else if (index == 217) {strcpy(volcano_name_buffer, "Little Sitkin");}
//     else if (index == 218) {strcpy(volcano_name_buffer, "Loihi");}
//     else if (index == 219) {strcpy(volcano_name_buffer, "Lokon-Empung");}
//     else if (index == 220) {strcpy(volcano_name_buffer, "Lolo");}
//     else if (index == 221) {strcpy(volcano_name_buffer, "Lolobau");}
//     else if (index == 222) {strcpy(volcano_name_buffer, "Loloru");}
//     else if (index == 223) {strcpy(volcano_name_buffer, "Long Island");}
//     else if (index == 224) {strcpy(volcano_name_buffer, "Longonot");}
//     else if (index == 225) {strcpy(volcano_name_buffer, "Lopevi");}
//     else if (index == 226) {strcpy(volcano_name_buffer, "Maderas");}
//     else if (index == 227) {strcpy(volcano_name_buffer, "Mageik");}
//     else if (index == 228) {strcpy(volcano_name_buffer, "Mahawu");}
//     else if (index == 229) {strcpy(volcano_name_buffer, "Makian");}
//     else if (index == 230) {strcpy(volcano_name_buffer, "Maly Semiachik");}
//     else if (index == 231) {strcpy(volcano_name_buffer, "Manam");}
//     else if (index == 232) {strcpy(volcano_name_buffer, "Mariveles");}
//     else if (index == 233) {strcpy(volcano_name_buffer, "Marsabit");}
//     else if (index == 234) {strcpy(volcano_name_buffer, "Masaya");}
//     else if (index == 235) {strcpy(volcano_name_buffer, "Mashkovtsev");}
//     else if (index == 236) {strcpy(volcano_name_buffer, "Mashu");}
//     else if (index == 237) {strcpy(volcano_name_buffer, "Mauna Kea");}
//     else if (index == 238) {strcpy(volcano_name_buffer, "Mauna Loa");}
//     else if (index == 239) {strcpy(volcano_name_buffer, "Mayon");}
//     else if (index == 240) {strcpy(volcano_name_buffer, "McDonald Islands");}
//     else if (index == 241) {strcpy(volcano_name_buffer, "McLoughlin");}
//     else if (index == 242) {strcpy(volcano_name_buffer, "McLoughlin");}
//     else if (index == 243) {strcpy(volcano_name_buffer, "Mega Basalt Field");}
//     else if (index == 244) {strcpy(volcano_name_buffer, "Mega Basalt Field");}
//     else if (index == 245) {strcpy(volcano_name_buffer, "Menan Buttes");}
//     else if (index == 246) {strcpy(volcano_name_buffer, "Menengai");}
//     else if (index == 247) {strcpy(volcano_name_buffer, "Merapi");}
//     else if (index == 248) {strcpy(volcano_name_buffer, "Merbabu");}
//     else if (index == 249) {strcpy(volcano_name_buffer, "Meru");}
//     else if (index == 250) {strcpy(volcano_name_buffer, "Methana");}
//     else if (index == 251) {strcpy(volcano_name_buffer, "Metis Shoal");}
//     else if (index == 252) {strcpy(volcano_name_buffer, "Michael");}
//     else if (index == 253) {strcpy(volcano_name_buffer, "Michoacan-Guanajuato");}
//     else if (index == 254) {strcpy(volcano_name_buffer, "Mílos");}
//     else if (index == 255) {strcpy(volcano_name_buffer, "Miyake-jima");}
//     else if (index == 256) {strcpy(volcano_name_buffer, "Moffett");}
//     else if (index == 257) {strcpy(volcano_name_buffer, "Mojanda");}
//     else if (index == 258) {strcpy(volcano_name_buffer, "Momotombo");}
//     else if (index == 259) {strcpy(volcano_name_buffer, "Monaco Bank");}
//     else if (index == 260) {strcpy(volcano_name_buffer, "Mono Lake Vol Field");}
//     else if (index == 261) {strcpy(volcano_name_buffer, "Monowai Seamount");}
//     else if (index == 262) {strcpy(volcano_name_buffer, "Moti");}
//     else if (index == 263) {strcpy(volcano_name_buffer, "Muria");}
//     else if (index == 264) {strcpy(volcano_name_buffer, "Nantai");}
//     else if (index == 265) {strcpy(volcano_name_buffer, "Nasu");}
//     else if (index == 266) {strcpy(volcano_name_buffer, "Navajo Volcanic Field");}
//     else if (index == 267) {strcpy(volcano_name_buffer, "Negro, Cerro");}
//     else if (index == 268) {strcpy(volcano_name_buffer, "Nemo Peak");}
//     else if (index == 269) {strcpy(volcano_name_buffer, "Nevis Peak");}
//     else if (index == 270) {strcpy(volcano_name_buffer, "Newberry");}
//     else if (index == 271) {strcpy(volcano_name_buffer, "Newer Volcanics Province");}
//     else if (index == 272) {strcpy(volcano_name_buffer, "Ngauruhoe");}
//     else if (index == 273) {strcpy(volcano_name_buffer, "Nikko-Shirane");}
//     else if (index == 274) {strcpy(volcano_name_buffer, "Nipesotsu-Maruyama");}
//     else if (index == 275) {strcpy(volcano_name_buffer, "Nisyros");}
//     else if (index == 276) {strcpy(volcano_name_buffer, "Nyambeni Hills");}
//     else if (index == 277) {strcpy(volcano_name_buffer, "Nyamuragira");}
//     else if (index == 278) {strcpy(volcano_name_buffer, "Nyiragongo");}
//     else if (index == 279) {strcpy(volcano_name_buffer, "Oahu");}
//     else if (index == 280) {strcpy(volcano_name_buffer, "Ojos del Salado");}
//     else if (index == 281) {strcpy(volcano_name_buffer, "Ojos del Salado");}
//     else if (index == 282) {strcpy(volcano_name_buffer, "Okataina");}
//     else if (index == 283) {strcpy(volcano_name_buffer, "Okmok");}
//     else if (index == 284) {strcpy(volcano_name_buffer, "Oku Volcanic Field");}
//     else if (index == 285) {strcpy(volcano_name_buffer, "Ol Doinyo Lengai");}
//     else if (index == 286) {strcpy(volcano_name_buffer, "Opala");}
//     else if (index == 287) {strcpy(volcano_name_buffer, "Oraefajokull");}
//     else if (index == 288) {strcpy(volcano_name_buffer, "Oshima");}
//     else if (index == 289) {strcpy(volcano_name_buffer, "Pacaya");}
//     else if (index == 290) {strcpy(volcano_name_buffer, "Pagan");}
//     else if (index == 291) {strcpy(volcano_name_buffer, "Pago");}
//     else if (index == 292) {strcpy(volcano_name_buffer, "Papandayan");}
//     else if (index == 293) {strcpy(volcano_name_buffer, "Paricutin");}
//     else if (index == 294) {strcpy(volcano_name_buffer, "Parinacota");}
//     else if (index == 295) {strcpy(volcano_name_buffer, "Parinacota");}
//     else if (index == 296) {strcpy(volcano_name_buffer, "Pauzhetka");}
//     else if (index == 297) {strcpy(volcano_name_buffer, "Pavlof");}
//     else if (index == 298) {strcpy(volcano_name_buffer, "Pavlof Sister");}
//     else if (index == 299) {strcpy(volcano_name_buffer, "Pelee");}
//     else if (index == 300) {strcpy(volcano_name_buffer, "Peuet Sague");}
//     else if (index == 301) {strcpy(volcano_name_buffer, "Pico");}
//     else if (index == 302) {strcpy(volcano_name_buffer, "Pilas, Las");}
//     else if (index == 303) {strcpy(volcano_name_buffer, "Pinacate");}
//     else if (index == 304) {strcpy(volcano_name_buffer, "Pinatubo");}
//     else if (index == 305) {strcpy(volcano_name_buffer, "Pisgah Lava Field");}
//     else if (index == 306) {strcpy(volcano_name_buffer, "Piton de la Fournaise");}
//     else if (index == 307) {strcpy(volcano_name_buffer, "Poas");}
//     else if (index == 308) {strcpy(volcano_name_buffer, "Popocatepetl");}
//     else if (index == 309) {strcpy(volcano_name_buffer, "Prevo Peak");}
//     else if (index == 310) {strcpy(volcano_name_buffer, "Purace");}
//     else if (index == 311) {strcpy(volcano_name_buffer, "Quill, The");}
//     else if (index == 312) {strcpy(volcano_name_buffer, "Rabaul");}
//     else if (index == 313) {strcpy(volcano_name_buffer, "Ragang");}
//     else if (index == 314) {strcpy(volcano_name_buffer, "Raikoke");}
//     else if (index == 315) {strcpy(volcano_name_buffer, "Rainier");}
//     else if (index == 316) {strcpy(volcano_name_buffer, "Rajabasa");}
//     else if (index == 317) {strcpy(volcano_name_buffer, "Raoul");}
//     else if (index == 318) {strcpy(volcano_name_buffer, "Rasshua");}
//     else if (index == 319) {strcpy(volcano_name_buffer, "Raung");}
//     else if (index == 320) {strcpy(volcano_name_buffer, "Rausu");}
//     else if (index == 321) {strcpy(volcano_name_buffer, "Redoubt");}
//     else if (index == 322) {strcpy(volcano_name_buffer, "Reventador");}
//     else if (index == 323) {strcpy(volcano_name_buffer, "Rincon de la Vieja");}
//     else if (index == 324) {strcpy(volcano_name_buffer, "Roundtop");}
//     else if (index == 325) {strcpy(volcano_name_buffer, "Ruapehu");}
//     else if (index == 326) {strcpy(volcano_name_buffer, "Rudakov");}
//     else if (index == 327) {strcpy(volcano_name_buffer, "Ruiz, Nevado del");}
//     else if (index == 328) {strcpy(volcano_name_buffer, "Rumble IV");}
//     else if (index == 329) {strcpy(volcano_name_buffer, "Saba");}
//     else if (index == 330) {strcpy(volcano_name_buffer, "Sabancaya");}
//     else if (index == 331) {strcpy(volcano_name_buffer, "Sakar");}
//     else if (index == 332) {strcpy(volcano_name_buffer, "Sakura-jima");}
//     else if (index == 333) {strcpy(volcano_name_buffer, "Salak");}
//     else if (index == 334) {strcpy(volcano_name_buffer, "San Cristobal");}
//     else if (index == 335) {strcpy(volcano_name_buffer, "San Salvador");}
//     else if (index == 336) {strcpy(volcano_name_buffer, "San Vicente");}
//     else if (index == 337) {strcpy(volcano_name_buffer, "Sanford");}
//     else if (index == 338) {strcpy(volcano_name_buffer, "Sangay");}
//     else if (index == 339) {strcpy(volcano_name_buffer, "Santa Ana");}
//     else if (index == 340) {strcpy(volcano_name_buffer, "Santa Maria");}
//     else if (index == 341) {strcpy(volcano_name_buffer, "Santorini");}
//     else if (index == 342) {strcpy(volcano_name_buffer, "Sarychev Peak");}
//     else if (index == 343) {strcpy(volcano_name_buffer, "Savo");}
//     else if (index == 344) {strcpy(volcano_name_buffer, "Segula");}
//     else if (index == 345) {strcpy(volcano_name_buffer, "Semeru");}
//     else if (index == 346) {strcpy(volcano_name_buffer, "Semisopochnoi");}
//     else if (index == 347) {strcpy(volcano_name_buffer, "Sete Cidades");}
//     else if (index == 348) {strcpy(volcano_name_buffer, "Shasta");}
//     else if (index == 349) {strcpy(volcano_name_buffer, "Sheveluch");}
//     else if (index == 350) {strcpy(volcano_name_buffer, "Shikotsu");}
//     else if (index == 351) {strcpy(volcano_name_buffer, "Shiretoko-Iwo-zan");}
//     else if (index == 352) {strcpy(volcano_name_buffer, "Shishaldin");}
//     else if (index == 353) {strcpy(volcano_name_buffer, "Sinarka");}
//     else if (index == 354) {strcpy(volcano_name_buffer, "Smirnov");}
//     else if (index == 355) {strcpy(volcano_name_buffer, "Sollipulli");}
//     else if (index == 356) {strcpy(volcano_name_buffer, "Soputan");}
//     else if (index == 357) {strcpy(volcano_name_buffer, "Soufriere Guadeloupe");}
//     else if (index == 358) {strcpy(volcano_name_buffer, "Soufrière Hills");}
//     else if (index == 359) {strcpy(volcano_name_buffer, "Soufrière St. Vincent");}
//     else if (index == 360) {strcpy(volcano_name_buffer, "South Island");}
//     else if (index == 361) {strcpy(volcano_name_buffer, "SP Mountain");}
//     else if (index == 362) {strcpy(volcano_name_buffer, "Spurr");}
//     else if (index == 363) {strcpy(volcano_name_buffer, "Srednii");}
//     else if (index == 364) {strcpy(volcano_name_buffer, "Steller");}
//     else if (index == 365) {strcpy(volcano_name_buffer, "Stromboli");}
//     else if (index == 366) {strcpy(volcano_name_buffer, "Sumaco");}
//     else if (index == 367) {strcpy(volcano_name_buffer, "Sumbing");}
//     else if (index == 368) {strcpy(volcano_name_buffer, "Sundoro");}
//     else if (index == 369) {strcpy(volcano_name_buffer, "Sunset Craters");}
//     else if (index == 370) {strcpy(volcano_name_buffer, "Suswa");}
//     else if (index == 371) {strcpy(volcano_name_buffer, "Suwanose-jima");}
//     else if (index == 372) {strcpy(volcano_name_buffer, "Taal");}
//     else if (index == 373) {strcpy(volcano_name_buffer, "Talang");}
//     else if (index == 374) {strcpy(volcano_name_buffer, "Tambora");}
//     else if (index == 375) {strcpy(volcano_name_buffer, "Tanaga and Takawangha");}
//     else if (index == 376) {strcpy(volcano_name_buffer, "Tangkubanparahu");}
//     else if (index == 377) {strcpy(volcano_name_buffer, "Tao-Rusyr Caldera");}
//     else if (index == 378) {strcpy(volcano_name_buffer, "Tarso Voon");}
//     else if (index == 379) {strcpy(volcano_name_buffer, "Telica Volcano");}
//     else if (index == 380) {strcpy(volcano_name_buffer, "Tenerife");}
//     else if (index == 381) {strcpy(volcano_name_buffer, "Tengger Caldera and Bromo");}
//     else if (index == 382) {strcpy(volcano_name_buffer, "Three Fingered Jack");}
//     else if (index == 383) {strcpy(volcano_name_buffer, "Three Sisters");}
//     else if (index == 384) {strcpy(volcano_name_buffer, "Thule Islands");}
//     else if (index == 385) {strcpy(volcano_name_buffer, "Tiatia");}
//     else if (index == 386) {strcpy(volcano_name_buffer, "Toba");}
//     else if (index == 387) {strcpy(volcano_name_buffer, "Todoko-Ranu");}
//     else if (index == 388) {strcpy(volcano_name_buffer, "Tokachi");}
//     else if (index == 389) {strcpy(volcano_name_buffer, "Tolbachik");}
//     else if (index == 390) {strcpy(volcano_name_buffer, "Tolimán");}
//     else if (index == 391) {strcpy(volcano_name_buffer, "Towada");}
//     else if (index == 392) {strcpy(volcano_name_buffer, "Traitor's Head");}
//     else if (index == 393) {strcpy(volcano_name_buffer, "Tri Sestry");}
//     else if (index == 394) {strcpy(volcano_name_buffer, "Tristan da Cunha");}
//     else if (index == 395) {strcpy(volcano_name_buffer, "Tsurumi");}
//     else if (index == 396) {strcpy(volcano_name_buffer, "Tungurahua");}
//     else if (index == 397) {strcpy(volcano_name_buffer, "Ubehebe Craters");}
//     else if (index == 398) {strcpy(volcano_name_buffer, "Ubinas");}
//     else if (index == 399) {strcpy(volcano_name_buffer, "Udina");}
//     else if (index == 400) {strcpy(volcano_name_buffer, "Ugashik and Peulik");}
//     else if (index == 401) {strcpy(volcano_name_buffer, "Uinkaret Volcanic Field");}
//     else if (index == 402) {strcpy(volcano_name_buffer, "Ulawun");}
//     else if (index == 403) {strcpy(volcano_name_buffer, "Umboi");}
//     else if (index == 404) {strcpy(volcano_name_buffer, "Undara");}
//     else if (index == 405) {strcpy(volcano_name_buffer, "Ungaran and Telomoyo");}
//     else if (index == 406) {strcpy(volcano_name_buffer, "Unzen");}
//     else if (index == 407) {strcpy(volcano_name_buffer, "Urataman");}
//     else if (index == 408) {strcpy(volcano_name_buffer, "Ushishur");}
//     else if (index == 409) {strcpy(volcano_name_buffer, "Usu");}
//     else if (index == 410) {strcpy(volcano_name_buffer, "Uzon");}
//     else if (index == 411) {strcpy(volcano_name_buffer, "Veniaminof");}
//     else if (index == 412) {strcpy(volcano_name_buffer, "Vernadskii Ridge");}
//     else if (index == 413) {strcpy(volcano_name_buffer, "Vesuvius");}
//     else if (index == 414) {strcpy(volcano_name_buffer, "Villarrica");}
//     else if (index == 415) {strcpy(volcano_name_buffer, "Vilyuchik");}
//     else if (index == 416) {strcpy(volcano_name_buffer, "Visoke");}
//     else if (index == 417) {strcpy(volcano_name_buffer, "Visoke");}
//     else if (index == 418) {strcpy(volcano_name_buffer, "Vsevidof and Recheschnoi Volcanoes");}
//     else if (index == 419) {strcpy(volcano_name_buffer, "Vulcano");}
//     else if (index == 420) {strcpy(volcano_name_buffer, "Washington");}
//     else if (index == 421) {strcpy(volcano_name_buffer, "Wau-en-Namus");}
//     else if (index == 422) {strcpy(volcano_name_buffer, "Westdahl");}
//     else if (index == 423) {strcpy(volcano_name_buffer, "White Island");}
//     else if (index == 424) {strcpy(volcano_name_buffer, "Wrangell");}
//     else if (index == 425) {strcpy(volcano_name_buffer, "Wudalianchi");}
//     else if (index == 426) {strcpy(volcano_name_buffer, "Yake-dake");}
//     else if (index == 427) {strcpy(volcano_name_buffer, "Yali");}
//     else if (index == 428) {strcpy(volcano_name_buffer, "Yantarni");}
//     else if (index == 429) {strcpy(volcano_name_buffer, "Yasur");}
//     else if (index == 430) {strcpy(volcano_name_buffer, "Yellowstone");}
//     else if (index == 431) {strcpy(volcano_name_buffer, "Zavaritzki Caldera");}
//     else if (index == 432) {strcpy(volcano_name_buffer, "Zheltovsky");}
//     else if (index == 433) {strcpy(volcano_name_buffer, "Zhupanovsky");}
//     else if (index == 434) {strcpy(volcano_name_buffer, "Zimina");}
//     else strcpy(volcano_name_buffer, "undefined");
//     return volcano_name_buffer;
// }

// // ----------------------------------------------------------------------------------------
// // Accessed by : volcano_index [0] via volcano_index_key.
// // Designed to require very low memory rather than having multiple instances per volcano.
// // ----------------------------------------------------------------------------------------
// const char volcano_locations[MAX_LOCATIONS][MAX_GLOBAL_ELEMENT_SIZE] = {
//     "Antarctica", "Argentina", "Argentina, Chile", "Australia", "Bolivia, Chile", "Cameroon",
//     "Cape Verde Islands", "Chad", "Chile", "China", "Colombia", "Comoros", "Congo, Rwanda",
//     "Costa Rica", "Democratic Republic of Congo", "Ecuador", "El Salvador", "Eritrea",
//     "Ethiopia", "Ethiopia, Kenya", "France", "Galápagos Islands", "Greece", "Grenada",
//     "Guatemala", "Iceland", "India", "Indonesia", "Iran", "Italy", "Japan", "Kenya", "Libya",
//     "Mariana Islands", "Mexico", "Netherlands", "New Zealand", "Nicaragua", "Norway",
//     "Pacific Ocean", "Papua New Guinea", "Peru", "Philippines", "Portugal", "Russia",
//     "Solomon Islands", "South Atlantic Ocean", "Spain", "St. Kitts and Nevis", "St. Vincent",
//     "Tanzania", "Tonga", "Turkey", "Uganda", "United Kingdom", "United States", "Vanuatu"
// };

// // ----------------------------------------------------------------------------------------
// // Accessed by : volcano_index [1] via volcano_index_key.
// // Designed to require very low memory rather than having multiple instances per volcano.
// // ----------------------------------------------------------------------------------------
// const char volcano_types[MAX_TYPES][MAX_GLOBAL_ELEMENT_SIZE] = {
//     "", "Caldera", "Caldera, Shield", "Cinder Cone", "Complex", "Fissure vent", "Fissure vent, Maar",
//     "Flood Basalt", "Lava cone, Stratovolcano", "Lava dome", "Maar", "Plug", "Pyroclastic cone",
//     "Shield", "Somma", "Stratovolcano", "Submarine", "Tuff ring", "Volcanic field"
// };

// // ----------------------------------------------------------------------------------------
// /*
//    Accessed by : volcano_index_key.
//    Elevation in meters above sea level.
// */
// // ----------------------------------------------------------------------------------------
// const int volcano_elevations[MAX_VOLCANOS] = {
//   641, // 0
//   6046, // 1
//   3976, // 2
//   1689, // 3
//   3742, // 4
//   -39, // 5
//   1718, // 6
//   1733, // 7
//   1295, // 8
//   965, // 9
//   3760, // 10
//   947, // 11
//   2546, // 12
//   3142, // 13
//   -137, // 14
//   1180, // 15
//   1828, // 16
//   1499, // 17
//   1956, // 18
//   1637, // 19
//   1366, // 20
//   584, // 21
//   1303, // 22
//   2339, // 23
//   744, // 24
//   1501, // 25
//   949, // 26
//   1130, // 27
//   1031, // 28
//   904, // 29
//   1334, // 30
//   1334, // 31
//   790, // 32
//   852, // 33
//   1341, // 34
//   5753, // 35
//   1496, // 36
//   5165, // 37
//   1670, // 38
//   5597, // 39
//   1910, // 40
//   2568, // 41
//   858, // 42
//   1516, // 43
//   1592, // 44
//   3535, // 45
//   1533, // 46
//   1206, // 47
//   1252, // 48
//   2741, // 49
//   1320, // 50
//   -1410, // 51
//   2145, // 52
//   5846, // 53
//   2035, // 54
//   1180, // 55
//   2763, // 56
//   1750, // 57
//   3285, // 58
//   685, // 59
//   2248, // 60
//   1819, // 61
//   305, // 62
//   1032, // 63
//   3490, // 64
//   2277, // 65
//   2096, // 66
//   2882, // 67
//   1187, // 68
//   1720, // 69
//   1565, // 70
//   2281, // 71
//   2281, // 72
//   4095, // 73
//   712, // 74
//   2435, // 75
//   2494, // 76
//   1730, // 77
//   5790, // 78
//   3078, // 79
//   1464, // 80
//   2067, // 81
//   5958, // 82
//   6310, // 83
//   1730, // 84
//   746, // 85
//   4100, // 86
//   0, // 87
//   1610, // 88
//   2965, // 89
//   847, // 90
//   5911, // 91
//   2471, // 92
//   2005, // 93
//   2290, // 94
//   400, // 95
//   5670, // 96
//   0, // 97
//   3173, // 98
//   1525, // 99
//   1558, // 100
//   1450, // 101
//   2565, // 102
//   1331, // 103
//   1625, // 104
//   1185, // 105
//   -34, // 106
//   1156, // 107
//   2518, // 108
//   788, // 109
//   1060, // 110
//   5633, // 111
//   3794, // 112
//   613, // 113
//   3350, // 114
//   2007, // 115
//   1476, // 116
//   1095, // 117
//   2829, // 118
//   1716, // 119
//   1920, // 120
//   3763, // 121
//   529, // 122
//   3776, // 123
//   805, // 124
//   1772, // 125
//   4276, // 126
//   2168, // 127
//   1715, // 128
//   2576, // 129
//   1635, // 130
//   1573, // 131
//   797, // 132
//   2958, // 133
//   3213, // 134
//   1829, // 135
//   402, // 136
//   1950, // 137
//   1740, // 138
//   2317, // 139
//   1725, // 140
//   4784, // 141
//   6071, // 142
//   287, // 143
//   1439, // 144
//   2745, // 145
//   1332, // 146
//   1500, // 147
//   3426, // 148
//   2523, // 149
//   1325, // 150
//   2386, // 151
//   3053, // 152
//   1659, // 153
//   5248, // 154
//   450, // 155
//   1578, // 156
//   161, // 157
//   1009, // 158
//   3431, // 159
//   2446, // 160
//   1950, // 161
//   5286, // 162
//   581, // 163
//   1130, // 164
//   3199, // 165
//   1400, // 166
//   365, // 167
//   901, // 168
//   450, // 169
//   2156, // 170
//   1307, // 171
//   1784, // 172
//   4507, // 173
//   4507, // 174
//   2361, // 175
//   1536, // 176
//   1512, // 177
//   2047, // 178
//   1067, // 179
//   -20, // 180
//   900, // 181
//   1731, // 182
//   3800, // 183
//   2090, // 184
//   1677, // 185
//   -185, // 186
//   704, // 187
//   1552, // 188
//   1222, // 189
//   5895, // 190
//   1700, // 191
//   4835, // 192
//   1131, // 193
//   1619, // 194
//   3456, // 195
//   1812, // 196
//   818, // 197
//   813, // 198
//   1856, // 199
//   3528, // 200
//   1079, // 201
//   2043, // 202
//   2171, // 203
//   999, // 204
//   581, // 205
//   -2, // 206
//   2426, // 207
//   1680, // 208
//   1330, // 209
//   670, // 210
//   5592, // 211
//   1117, // 212
//   1703, // 213
//   1423, // 214
//   1156, // 215
//   602, // 216
//   1174, // 217
//   -975, // 218
//   1580, // 219
//   805, // 220
//   858, // 221
//   1887, // 222
//   1280, // 223
//   2776, // 224
//   1413, // 225
//   1394, // 226
//   2165, // 227
//   1324, // 228
//   1357, // 229
//   1560, // 230
//   1807, // 231
//   1388, // 232
//   1707, // 233
//   635, // 234
//   503, // 235
//   855, // 236
//   4205, // 237
//   4170, // 238
//   2462, // 239
//   230, // 240
//   2894, // 241
//   2894, // 242
//   1067, // 243
//   1067, // 244
//   1713, // 245
//   2278, // 246
//   2911, // 247
//   3145, // 248
//   4565, // 249
//   760, // 250
//   43, // 251
//   990, // 252
//   3860, // 253
//   751, // 254
//   815, // 255
//   1196, // 256
//   4263, // 257
//   1297, // 258
//   -197, // 259
//   2121, // 260
//   -132, // 261
//   950, // 262
//   1625, // 263
//   2486, // 264
//   1915, // 265
//   2740, // 266
//   728, // 267
//   1018, // 268
//   985, // 269
//   2434, // 270
//   1011, // 271
//   1978, // 272
//   2578, // 273
//   2013, // 274
//   698, // 275
//   750, // 276
//   3058, // 277
//   3470, // 278
//   1220, // 279
//   6887, // 280
//   6887, // 281
//   1111, // 282
//   1073, // 283
//   3011, // 284
//   2962, // 285
//   1776, // 286
//   2119, // 287
//   764, // 288
//   2552, // 289
//   570, // 290
//   742, // 291
//   2665, // 292
//   3170, // 293
//   6348, // 294
//   6348, // 295
//   1070, // 296
//   2519, // 297
//   2142, // 298
//   1397, // 299
//   2801, // 300
//   2351, // 301
//   1700, // 302
//   1200, // 303
//   1486, // 304
//   1495, // 305
//   2631, // 306
//   2708, // 307
//   5246, // 308
//   1360, // 309
//   4650, // 310
//   601, // 311
//   688, // 312
//   2815, // 313
//   551, // 314
//   4392, // 315
//   1281, // 316
//   516, // 317
//   956, // 318
//   3332, // 319
//   1660, // 320
//   3108, // 321
//   3562, // 322
//   1916, // 323
//   1871, // 324
//   2797, // 325
//   542, // 326
//   5321, // 327
//   500, // 328
//   887, // 329
//   5967, // 330
//   992, // 331
//   1117, // 332
//   2211, // 333
//   1745, // 334
//   1893, // 335
//   697, // 336
//   4949, // 337
//   5230, // 338
//   2381, // 339
//   3772, // 340
//   367, // 341
//   1496, // 342
//   485, // 343
//   1160, // 344
//   3676, // 345
//   1221, // 346
//   856, // 347
//   4317, // 348
//   3283, // 349
//   1320, // 350
//   1563, // 351
//   2857, // 352
//   934, // 353
//   1189, // 354
//   2282, // 355
//   1784, // 356
//   1467, // 357
//   915, // 358
//   1220, // 359
//   800, // 360
//   2141, // 361
//   3374, // 362
//   36, // 363
//   2272, // 364
//   924, // 365
//   3990, // 366
//   3371, // 367
//   3136, // 368
//   2447, // 369
//   2356, // 370
//   799, // 371
//   311, // 372
//   2597, // 373
//   2850, // 374
//   1806, // 375
//   2084, // 376
//   1325, // 377
//   3100, // 378
//   1010, // 379
//   3715, // 380
//   2329, // 381
//   2390, // 382
//   3100, // 383
//   1075, // 384
//   1819, // 385
//   2157, // 386
//   979, // 387
//   2077, // 388
//   3682, // 389
//   3158, // 390
//   1159, // 391
//   1881, // 392
//   998, // 393
//   2060, // 394
//   1584, // 395
//   2011, // 396
//   752, // 397
//   5672, // 398
//   2923, // 399
//   1474, // 400
//   1555, // 401
//   2334, // 402
//   1548, // 403
//   1020, // 404
//   2050, // 405
//   1500, // 406
//   678, // 407
//   401, // 408
//   737, // 409
//   1617, // 410
//   2507, // 411
//   1183, // 412
//   1281, // 413
//   2847, // 414
//   2173, // 415
//   3711, // 416
//   3711, // 417
//   2149, // 418
//   500, // 419
//   2376, // 420
//   547, // 421
//   1654, // 422
//   321, // 423
//   4317, // 424
//   597, // 425
//   2455, // 426
//   180, // 427
//   1345, // 428
//   361, // 429
//   2805, // 430
//   624, // 431
//   1953, // 432
//   2958, // 433
//   3081, // 434
// };

// // ----------------------------------------------------------------------------------------
// /*
//    Accessed by : volcano_index_key.
//    0 : latitude
//    1 : longitude
//    These are not currently all accurate however a new array is being developed that is 100%
//    accurate and will require a double array rather than float array.
// */
// // ----------------------------------------------------------------------------------------
// const float volcano_lat_long[MAX_VOLCANOS][2] = {
//   {34.5,131.6}, // 0
//   {-23.3,-67.62}, // 1
//   {14.5,-90.88}, // 2
//   {38.57,34.52}, // 3
//   {46.21,-121.49}, // 4
//   {-25.37,-129.27}, // 5
//   {37.64,140.29}, // 6
//   {10.07,40.84}, // 7
//   {13.08,40.85}, // 8
//   {18.77,145.67}, // 9
//   {14.47,-90.74}, // 10
//   {37.77,-25.47}, // 11
//   {-50.33,-73.75}, // 12
//   {-8.34,115.51}, // 13
//   {20.42,145.03}, // 14
//   {53.98,159.45}, // 15
//   {36.56,139.2}, // 16
//   {43.38,144.01}, // 17
//   {55.43,158.65}, // 18
//   {39.76,140.8}, // 19
//   {39.96,140.76}, // 20
//   {29.46,129.6}, // 21
//   {54.13,-165.99}, // 22
//   {50.86,155.55}, // 23
//   {17.6,145.83}, // 24
//   {12.88,40.57}, // 25
//   {41.73,12.7}, // 26
//   {-0.43,-91.12}, // 27
//   {13.52,40.63}, // 28
//   {14.88,39.92}, // 29
//   {-16.25,168.12}, // 30
//   {-16.25,168.12}, // 31
//   {16.35,145.67}, // 32
//   {-20.2,169.78}, // 33
//   {56.88,-158.17}, // 34
//   {-0.48,-78.14}, // 35
//   {-15.4,167.83}, // 36
//   {39.7,44.3}, // 37
//   {10.46,-84.7}, // 38
//   {-18.73,-69.05}, // 39
//   {52.36,157.83}, // 40
//   {36.4,138.53}, // 41
//   {-7.95,-14.37}, // 42
//   {65.03,-16.75}, // 43
//   {32.88,131.11}, // 44
//   {14.58,-91.19}, // 45
//   {52.38,-174.15}, // 46
//   {44.81,147.14}, // 47
//   {59.36,-153.43}, // 48
//   {53.26,158.83}, // 49
//   {3.67,125.5}, // 50
//   {45.95,-130.0}, // 51
//   {10.08,40.7}, // 52
//   {-21.79,-68.24}, // 53
//   {37.73,140.25}, // 54
//   {19.5,121.9}, // 55
//   {43.8,121.6}, // 56
//   {-6.1,155.2}, // 57
//   {48.79,-121.82}, // 58
//   {-3.6,144.85}, // 59
//   {-5.2,151.2}, // 60
//   {37.6,140.1}, // 61
//   {12.29,93.88}, // 62
//   {2.32,36.57}, // 63
//   {28.0,60.0}, // 64
//   {71.1,-8.2}, // 65
//   {44.28,-121.83}, // 66
//   {55.98,160.58}, // 67
//   {11.5,124.5}, // 68
//   {54.32,160.02}, // 69
//   {12.8,124.1}, // 70
//   {8.05,38.35}, // 71
//   {8.05,38.35}, // 72
//   {4.2,9.17}, // 73
//   {18.83,121.85}, // 74
//   {10.41,123.13}, // 75
//   {36.45,-104.09}, // 76
//   {33.7,-106.0}, // 77
//   {0.03,-77.99}, // 78
//   {-6.9,108.41}, // 79
//   {45.78,2.97}, // 80
//   {57.13,-156.99}, // 81
//   {50.33,155.46}, // 82
//   {-1.46,-78.82}, // 83
//   {52.83,-169.95}, // 84
//   {13.86,-89.54}, // 85
//   {19.51,103.62}, // 86
//   {NAN,NAN}, // 87
//   {11.5,85.6}, // 88
//   {-37.85,-71.17}, // 89
//   {12.98,-87.56}, // 90
//   {-0.68,-78.44}, // 91
//   {42.94,-122.11}, // 92
//   {43.42,-113.5}, // 93
//   {43.7,142.9}, // 94
//   {-5.0,150.1}, // 95
//   {35.95,52.11}, // 96
//   {NAN,NAN}, // 97
//   {-4.0,103.1}, // 98
//   {43.5,-120.9}, // 99
//   {44.6,-104.7}, // 100
//   {43.1,-118.7}, // 101
//   {-7.2,109.9}, // 102
//   {51.46,156.98}, // 103
//   {13.6,41.8}, // 104
//   {1.7,127.87}, // 105
//   {-16.7,168.4}, // 106
//   {50.68,156.02}, // 107
//   {-39.3,174.1}, // 108
//   {-8.7,122.45}, // 109
//   {17.36,-93.23}, // 110
//   {43.33,42.45}, // 111
//   {-77.5,167.2}, // 112
//   {13.6,40.7}, // 113
//   {37.73,15.0}, // 114
//   {8.98,39.93}, // 115
//   {-0.37,-91.55}, // 116
//   {-54.63,-164.42}, // 117
//   {14.95,-24.35}, // 118
//   {43.37,-121.07}, // 119
//   {55.08,-162.81}, // 120
//   {14.47,-90.88}, // 121
//   {28.36,-14.02}, // 122
//   {35.36,138.73}, // 123
//   {37.76,-25.33}, // 124
//   {50.27,155.25}, // 125
//   {1.22,-77.37}, // 126
//   {-7.26,108.08}, // 127
//   {0.81,127.33}, // 128
//   {54.97,160.68}, // 129
//   {1.38,127.53}, // 130
//   {51.8,178.8}, // 131
//   {-14.27,167.5}, // 132
//   {-6.77,106.94}, // 133
//   {48.11,-121.11}, // 134
//   {52.56,158.03}, // 135
//   {39.02,-27.97}, // 136
//   {28.0,15.58}, // 137
//   {52.07,-176.13}, // 138
//   {58.35,-155.09}, // 139
//   {64.42,-17.33}, // 140
//   {-0.17,-78.6}, // 141
//   {-18.42,-69.09}, // 142
//   {17.31,145.84}, // 143
//   {35.23,139.02}, // 144
//   {-53.1,73.5}, // 145
//   {9.2,124.7}, // 146
//   {27.73,-18.03}, // 147
//   {45.4,-121.7}, // 148
//   {19.69,-155.87}, // 149
//   {1.49,127.63}, // 150
//   {-8.06,114.24}, // 151
//   {60.03,-153.09}, // 152
//   {-8.34,123.26}, // 153
//   {-0.66,-78.71}, // 154
//   {13.67,-89.05}, // 155
//   {51.49,157.2}, // 156
//   {24.75,141.29}, // 157
//   {20.47,122.01}, // 158
//   {9.98,83.85}, // 159
//   {54.77,-163.72}, // 160
//   {13.81,-89.63}, // 161
//   {19.18,-98.64}, // 162
//   {34.9,139.1}, // 163
//   {1.08,127.42}, // 164
//   {44.67,-121.8}, // 165
//   {43.1,-117.4}, // 166
//   {-3.63,144.63}, // 167
//   {58.62,-154.06}, // 168
//   {20.55,-156.57}, // 169
//   {51.3,156.87}, // 170
//   {51.92,-177.17}, // 171
//   {2.78,125.4}, // 172
//   {-1.51,29.45}, // 173
//   {-1.51,29.45}, // 174
//   {-11.75,43.38}, // 175
//   {54.05,159.45}, // 176
//   {63.63,-19.05}, // 177
//   {58.28,-154.96}, // 178
//   {-0.08,29.92}, // 179
//   {-9.02,157.95}, // 180
//   {51.65,157.35}, // 181
//   {-7.93,112.31}, // 182
//   {1.7,101.26}, // 183
//   {52.06,157.7}, // 184
//   {57.2,-156.75}, // 185
//   {12.3,-61.64}, // 186
//   {30.79,130.31}, // 187
//   {54.49,160.25}, // 188
//   {19.42,-155.29}, // 189
//   {3.07,37.35}, // 190
//   {31.93,130.86}, // 191
//   {56.06,160.64}, // 192
//   {42.06,140.68}, // 193
//   {8.8,39.69}, // 194
//   {53.32,158.69}, // 195
//   {51.36,156.75}, // 196
//   {65.73,-16.78}, // 197
//   {-6.1,105.42}, // 198
//   {54.59,160.27}, // 199
//   {54.75,160.53}, // 200
//   {51.8,157.53}, // 201
//   {58.45,-154.36}, // 202
//   {36.64,138.54}, // 203
//   {43.61,144.44}, // 204
//   {42.49,141.16}, // 205
//   {-16.83,168.54}, // 206
//   {28.57,-17.83}, // 207
//   {-8.95,148.15}, // 208
//   {-5.53,148.42}, // 209
//   {29.03,-13.63}, // 210
//   {-23.37,-67.73}, // 211
//   {-8.36,122.84}, // 212
//   {-8.54,122.78}, // 213
//   {-8.27,123.51}, // 214
//   {17.37,-62.8}, // 215
//   {38.48,14.95}, // 216
//   {51.95,178.54}, // 217
//   {18.92,-155.27}, // 218
//   {1.36,124.79}, // 219
//   {-5.47,150.51}, // 220
//   {-4.92,151.16}, // 221
//   {-6.52,155.62}, // 222
//   {-5.36,147.12}, // 223
//   {-0.91,36.45}, // 224
//   {-16.51,168.35}, // 225
//   {11.45,-85.52}, // 226
//   {58.2,155.25}, // 227
//   {1.36,124.86}, // 228
//   {0.32,127.4}, // 229
//   {54.13,159.67}, // 230
//   {-4.08,145.04}, // 231
//   {14.52,120.47}, // 232
//   {2.32,37.97}, // 233
//   {11.98,-86.16}, // 234
//   {51.1,156.72}, // 235
//   {43.57,144.57}, // 236
//   {19.82,-155.47}, // 237
//   {19.48,-155.61}, // 238
//   {13.26,123.69}, // 239
//   {-53.03,72.6}, // 240
//   {42.45,-122.32}, // 241
//   {42.45,-122.32}, // 242
//   {37.42,4.08}, // 243
//   {37.42,4.08}, // 244
//   {43.7,-111.96}, // 245
//   {-0.2,36.07}, // 246
//   {-7.54,110.44}, // 247
//   {-7.45,110.43}, // 248
//   {-3.25,36.75}, // 249
//   {37.62,23.34}, // 250
//   {-19.18,-174.87}, // 251
//   {-57.78,-26.45}, // 252
//   {19.85,-101.75}, // 253
//   {36.7,24.44}, // 254
//   {34.08,139.53}, // 255
//   {51.94,-176.75}, // 256
//   {0.13,-78.27}, // 257
//   {12.42,-86.54}, // 258
//   {37.6,-25.88}, // 259
//   {38.0,-119.03}, // 260
//   {-25.89,-177.19}, // 261
//   {0.45,127.4}, // 262
//   {-6.62,110.88}, // 263
//   {36.76,139.48}, // 264
//   {37.12,139.97}, // 265
//   {NAN,NAN}, // 266
//   {12.51,-86.7}, // 267
//   {49.57,154.81}, // 268
//   {17.15,-62.58}, // 269
//   {43.72,-121.23}, // 270
//   {-37.77,142.5}, // 271
//   {-39.13,175.64}, // 272
//   {36.8,139.38}, // 273
//   {43.45,143.04}, // 274
//   {36.59,27.16}, // 275
//   {0.23,37.87}, // 276
//   {-1.41,29.2}, // 277
//   {-1.52,29.25}, // 278
//   {NAN,NAN}, // 279
//   {-27.12,-68.55}, // 280
//   {-27.12,-68.55}, // 281
//   {-38.12,176.5}, // 282
//   {53.43,-168.13}, // 283
//   {6.25,10.5}, // 284
//   {-2.76,39.91}, // 285
//   {52.54,157.34}, // 286
//   {64.0,-16.65}, // 287
//   {34.72,139.4}, // 288
//   {14.38,-90.6}, // 289
//   {18.13,145.8}, // 290
//   {-5.58,150.52}, // 291
//   {-7.32,107.73}, // 292
//   {19.5,-102.2}, // 293
//   {-18.17,-69.15}, // 294
//   {-18.17,-69.15}, // 295
//   {51.45,156.97}, // 296
//   {55.42,-161.89}, // 297
//   {55.45,-161.84}, // 298
//   {14.82,-61.17}, // 299
//   {4.91,96.33}, // 300
//   {38.47,-28.4}, // 301
//   {11.54,-85.62}, // 302
//   {31.77,-113.5}, // 303
//   {15.13,120.35}, // 304
//   {34.75,-116.63}, // 305
//   {-21.23,55.71}, // 306
//   {10.2,-84.23}, // 307
//   {19.02,-98.62}, // 308
//   {47.02,152.12}, // 309
//   {2.32,-76.4}, // 310
//   {17.48,-62.96}, // 311
//   {-4.27,152.2}, // 312
//   {7.7,124.5}, // 313
//   {48.29,153.25}, // 314
//   {46.85,-121.76}, // 315
//   {-5.78,105.63}, // 316
//   {-29.27,-177.92}, // 317
//   {47.77,153.02}, // 318
//   {-8.13,114.04}, // 319
//   {44.07,145.13}, // 320
//   {60.49,-152.74}, // 321
//   {-0.08,-77.66}, // 322
//   {10.83,-85.32}, // 323
//   {54.8,-163.59}, // 324
//   {-39.28,175.57}, // 325
//   {45.88,149.83}, // 326
//   {4.89,-75.32}, // 327
//   {-36.13,178.05}, // 328
//   {17.63,-63.23}, // 329
//   {-15.78,-71.85}, // 330
//   {-5.41,148.09}, // 331
//   {31.59,130.66}, // 332
//   {-6.72,106.73}, // 333
//   {12.7,-87.0}, // 334
//   {13.73,-89.29}, // 335
//   {16.9,-25.0}, // 336
//   {62.22,-144.13}, // 337
//   {-2.0,-78.34}, // 338
//   {13.85,-89.63}, // 339
//   {14.76,-91.55}, // 340
//   {36.4,25.4}, // 341
//   {48.09,153.2}, // 342
//   {-9.13,159.82}, // 343
//   {52.02,178.14}, // 344
//   {-8.11,112.92}, // 345
//   {51.93,179.58}, // 346
//   {37.87,-25.78}, // 347
//   {41.41,-122.19}, // 348
//   {56.65,161.36}, // 349
//   {42.69,141.38}, // 350
//   {44.13,145.17}, // 351
//   {54.76,-163.97}, // 352
//   {48.88,154.18}, // 353
//   {44.42,146.14}, // 354
//   {-38.97,-71.52}, // 355
//   {1.11,124.73}, // 356
//   {16.05,-61.67}, // 357
//   {16.72,-62.18}, // 358
//   {13.33,-61.18}, // 359
//   {2.63,36.6}, // 360
//   {35.6,-111.6}, // 361
//   {61.3,-152.25}, // 362
//   {47.6,152.92}, // 363
//   {58.4,-154.4}, // 364
//   {38.79,15.21}, // 365
//   {-0.54,-77.63}, // 366
//   {-7.38,110.07}, // 367
//   {-7.3,109.99}, // 368
//   {35.37,-111.5}, // 369
//   {-1.18,36.35}, // 370
//   {29.64,129.72}, // 371
//   {14.0,120.99}, // 372
//   {-0.98,100.68}, // 373
//   {-8.25,118.0}, // 374
//   {51.89,-178.15}, // 375
//   {-6.77,107.6}, // 376
//   {49.35,154.7}, // 377
//   {20.92,17.28}, // 378
//   {12.6,-86.85}, // 379
//   {28.27,-16.64}, // 380
//   {-7.94,112.95}, // 381
//   {44.5,-121.8}, // 382
//   {44.1,121.77}, // 383
//   {-59.45,-27.37}, // 384
//   {44.35,146.26}, // 385
//   {2.58,98.83}, // 386
//   {1.25,127.47}, // 387
//   {43.42,142.69}, // 388
//   {55.83,160.33}, // 389
//   {14.61,-91.19}, // 390
//   {40.47,140.92}, // 391
//   {-18.75,169.23}, // 392
//   {45.93,149.92}, // 393
//   {-37.09,-12.28}, // 394
//   {33.28,131.43}, // 395
//   {-1.47,-78.44}, // 396
//   {37.02,-117.45}, // 397
//   {-16.36,-70.9}, // 398
//   {55.76,160.53}, // 399
//   {57.75,-156.37}, // 400
//   {36.38,-113.13}, // 401
//   {-5.05,151.33}, // 402
//   {-5.59,147.88}, // 403
//   {-18.25,144.75}, // 404
//   {-7.18,110.33}, // 405
//   {32.76,130.29}, // 406
//   {47.12,152.25}, // 407
//   {47.52,152.8}, // 408
//   {42.54,140.84}, // 409
//   {54.5,159.97}, // 410
//   {56.17,-159.38}, // 411
//   {50.55,155.97}, // 412
//   {40.82,14.43}, // 413
//   {-39.42,-71.93}, // 414
//   {52.7,158.28}, // 415
//   {-1.47,29.49}, // 416
//   {-1.47,29.49}, // 417
//   {53.13,-168.69}, // 418
//   {38.4,14.96}, // 419
//   {44.3,-121.8}, // 420
//   {25.05,17.55}, // 421
//   {54.52,-164.65}, // 422
//   {-37.52,177.18}, // 423
//   {62.0,-144.02}, // 424
//   {48.72,126.12}, // 425
//   {36.22,137.59}, // 426
//   {36.67,27.14}, // 427
//   {57.02,-157.19}, // 428
//   {-19.53,169.44}, // 429
//   {44.43,-110.67}, // 430
//   {46.93,151.95}, // 431
//   {51.57,157.32}, // 432
//   {53.59,159.15}, // 433
//   {55.86,160.6}, // 434
// };

// // ----------------------------------------------------------------------------------------
// /*
//    Accessed by : volcano_index_key.
//    0: location index
//    1: type index
//    Designed to fascilitate very low memory requirements to provide location and type.
// */
// // ----------------------------------------------------------------------------------------
// const int volcano_index[MAX_VOLCANOS][2] = {
//   {30,13}, // 0
//   {8,15}, // 1
//   {24,15}, // 2
//   {52,1}, // 3
//   {55,15}, // 4
//   {39,16}, // 5
//   {30,15}, // 6
//   {18,15}, // 7
//   {18,15}, // 8
//   {33,15}, // 9
//   {24,15}, // 10
//   {43,15}, // 11
//   {8,15}, // 12
//   {27,15}, // 13
//   {33,16}, // 14
//   {44,1}, // 15
//   {30,15}, // 16
//   {30,1}, // 17
//   {44,13}, // 18
//   {30,15}, // 19
//   {30,15}, // 20
//   {30,15}, // 21
//   {55,15}, // 22
//   {44,15}, // 23
//   {33,15}, // 24
//   {18,13}, // 25
//   {29,1}, // 26
//   {21,13}, // 27
//   {18,15}, // 28
//   {17,15}, // 29
//   {56,2}, // 30
//   {56,2}, // 31
//   {33,15}, // 32
//   {56,15}, // 33
//   {55,1}, // 34
//   {15,15}, // 35
//   {56,13}, // 36
//   {52,15}, // 37
//   {13,15}, // 38
//   {8,4}, // 39
//   {44,15}, // 40
//   {30,4}, // 41
//   {46,15}, // 42
//   {25,15}, // 43
//   {30,1}, // 44
//   {24,4}, // 45
//   {55,1}, // 46
//   {44,15}, // 47
//   {55,9}, // 48
//   {44,15}, // 49
//   {27,15}, // 50
//   {39,16}, // 51
//   {18,15}, // 52
//   {8,4}, // 53
//   {30,15}, // 54
//   {42,15}, // 55
//   {55,15}, // 56
//   {40,3}, // 57
//   {55,15}, // 58
//   {40,15}, // 59
//   {40,15}, // 60
//   {30,15}, // 61
//   {26,12}, // 62
//   {31,4}, // 63
//   {28,15}, // 64
//   {38,15}, // 65
//   {55,13}, // 66
//   {44,15}, // 67
//   {42,4}, // 68
//   {44,15}, // 69
//   {42,15}, // 70
//   {18,6}, // 71
//   {18,6}, // 72
//   {5,15}, // 73
//   {42,15}, // 74
//   {42,15}, // 75
//   {55,3}, // 76
//   {55,18}, // 77
//   {15,15}, // 78
//   {27,15}, // 79
//   {20,9}, // 80
//   {55,15}, // 81
//   {44,15}, // 82
//   {15,15}, // 83
//   {55,15}, // 84
//   {16,1}, // 85
//   {34,15}, // 86
//   {55,7}, // 87
//   {37,15}, // 88
//   {1,15}, // 89
//   {37,13}, // 90
//   {15,15}, // 91
//   {55,1}, // 92
//   {55,18}, // 93
//   {30,15}, // 94
//   {40,1}, // 95
//   {28,15}, // 96
//   {26,7}, // 97
//   {27,15}, // 98
//   {55,18}, // 99
//   {55,11}, // 100
//   {55,18}, // 101
//   {27,4}, // 102
//   {44,9}, // 103
//   {17,15}, // 104
//   {27,4}, // 105
//   {56,1}, // 106
//   {44,14}, // 107
//   {36,15}, // 108
//   {27,15}, // 109
//   {34,1}, // 110
//   {44,15}, // 111
//   {0,15}, // 112
//   {18,13}, // 113
//   {29,15}, // 114
//   {18,15}, // 115
//   {15,1}, // 116
//   {55,15}, // 117
//   {6,15}, // 118
//   {55,10}, // 119
//   {55,15}, // 120
//   {24,15}, // 121
//   {47,5}, // 122
//   {30,15}, // 123
//   {43,15}, // 124
//   {44,15}, // 125
//   {10,15}, // 126
//   {27,15}, // 127
//   {27,15}, // 128
//   {44,15}, // 129
//   {27,15}, // 130
//   {55,15}, // 131
//   {56,15}, // 132
//   {27,0}, // 133
//   {55,15}, // 134
//   {44,1}, // 135
//   {43,15}, // 136
//   {47,5}, // 137
//   {55,15}, // 138
//   {55,15}, // 139
//   {25,1}, // 140
//   {15,0}, // 141
//   {8,15}, // 142
//   {55,15}, // 143
//   {30,0}, // 144
//   {3,15}, // 145
//   {42,15}, // 146
//   {47,13}, // 147
//   {55,15}, // 148
//   {55,13}, // 149
//   {27,15}, // 150
//   {27,15}, // 151
//   {55,15}, // 152
//   {27,15}, // 153
//   {15,15}, // 154
//   {16,1}, // 155
//   {44,15}, // 156
//   {30,1}, // 157
//   {42,15}, // 158
//   {13,15}, // 159
//   {55,15}, // 160
//   {16,15}, // 161
//   {34,15}, // 162
//   {30,12}, // 163
//   {27,15}, // 164
//   {55,15}, // 165
//   {55,18}, // 166
//   {40,15}, // 167
//   {55,15}, // 168
//   {55,13}, // 169
//   {44,15}, // 170
//   {55,15}, // 171
//   {27,15}, // 172
//   {12,15}, // 173
//   {12,15}, // 174
//   {11,13}, // 175
//   {44,15}, // 176
//   {25,5}, // 177
//   {55,15}, // 178
//   {53,18}, // 179
//   {45,16}, // 180
//   {44,15}, // 181
//   {27,15}, // 182
//   {27,15}, // 183
//   {44,15}, // 184
//   {55,15}, // 185
//   {23,16}, // 186
//   {30,1}, // 187
//   {44,15}, // 188
//   {55,13}, // 189
//   {50,15}, // 190
//   {30,15}, // 191
//   {44,15}, // 192
//   {30,15}, // 193
//   {18,1}, // 194
//   {44,15}, // 195
//   {44,15}, // 196
//   {25,1}, // 197
//   {27,15}, // 198
//   {44,15}, // 199
//   {44,15}, // 200
//   {44,15}, // 201
//   {55,15}, // 202
//   {30,15}, // 203
//   {30,1}, // 204
//   {30,15}, // 205
//   {56,1}, // 206
//   {47,15}, // 207
//   {40,15}, // 208
//   {40,15}, // 209
//   {47,5}, // 210
//   {8,15}, // 211
//   {27,4}, // 212
//   {27,15}, // 213
//   {27,15}, // 214
//   {48,15}, // 215
//   {29,15}, // 216
//   {55,15}, // 217
//   {55,16}, // 218
//   {27,15}, // 219
//   {40,15}, // 220
//   {40,1}, // 221
//   {40,15}, // 222
//   {40,4}, // 223
//   {31,15}, // 224
//   {56,15}, // 225
//   {37,15}, // 226
//   {55,15}, // 227
//   {27,15}, // 228
//   {27,15}, // 229
//   {44,1}, // 230
//   {40,15}, // 231
//   {42,15}, // 232
//   {31,13}, // 233
//   {37,1}, // 234
//   {44,15}, // 235
//   {30,1}, // 236
//   {55,13}, // 237
//   {55,13}, // 238
//   {42,15}, // 239
//   {3,4}, // 240
//   {55,8}, // 241
//   {55,8}, // 242
//   {19,12}, // 243
//   {19,12}, // 244
//   {55,17}, // 245
//   {31,13}, // 246
//   {27,15}, // 247
//   {27,15}, // 248
//   {50,15}, // 249
//   {22,9}, // 250
//   {51,16}, // 251
//   {54,15}, // 252
//   {34,3}, // 253
//   {22,15}, // 254
//   {30,15}, // 255
//   {55,15}, // 256
//   {15,15}, // 257
//   {37,15}, // 258
//   {43,16}, // 259
//   {55,18}, // 260
//   {36,16}, // 261
//   {27,15}, // 262
//   {27,15}, // 263
//   {30,15}, // 264
//   {30,15}, // 265
//   {55,18}, // 266
//   {37,3}, // 267
//   {44,1}, // 268
//   {48,15}, // 269
//   {55,13}, // 270
//   {3,13}, // 271
//   {36,15}, // 272
//   {30,13}, // 273
//   {30,15}, // 274
//   {22,15}, // 275
//   {31,13}, // 276
//   {14,13}, // 277
//   {14,15}, // 278
//   {55,13}, // 279
//   {2,15}, // 280
//   {2,15}, // 281
//   {36,9}, // 282
//   {55,13}, // 283
//   {5,15}, // 284
//   {50,15}, // 285
//   {44,1}, // 286
//   {25,15}, // 287
//   {30,15}, // 288
//   {24,4}, // 289
//   {55,15}, // 290
//   {40,1}, // 291
//   {27,15}, // 292
//   {34,3}, // 293
//   {4,15}, // 294
//   {4,15}, // 295
//   {44,9}, // 296
//   {55,15}, // 297
//   {55,15}, // 298
//   {20,15}, // 299
//   {27,4}, // 300
//   {43,15}, // 301
//   {37,15}, // 302
//   {34,3}, // 303
//   {42,15}, // 304
//   {55,18}, // 305
//   {20,13}, // 306
//   {13,15}, // 307
//   {34,15}, // 308
//   {44,15}, // 309
//   {10,15}, // 310
//   {35,15}, // 311
//   {40,13}, // 312
//   {42,15}, // 313
//   {44,15}, // 314
//   {55,15}, // 315
//   {27,15}, // 316
//   {36,15}, // 317
//   {44,15}, // 318
//   {27,15}, // 319
//   {30,15}, // 320
//   {55,15}, // 321
//   {15,15}, // 322
//   {13,4}, // 323
//   {55,15}, // 324
//   {36,15}, // 325
//   {44,15}, // 326
//   {10,15}, // 327
//   {36,16}, // 328
//   {35,15}, // 329
//   {41,15}, // 330
//   {40,15}, // 331
//   {30,15}, // 332
//   {27,15}, // 333
//   {37,15}, // 334
//   {16,15}, // 335
//   {6,15}, // 336
//   {55,13}, // 337
//   {15,15}, // 338
//   {16,15}, // 339
//   {24,15}, // 340
//   {22,13}, // 341
//   {44,15}, // 342
//   {45,15}, // 343
//   {55,15}, // 344
//   {27,15}, // 345
//   {55,15}, // 346
//   {43,15}, // 347
//   {55,15}, // 348
//   {44,15}, // 349
//   {30,1}, // 350
//   {30,15}, // 351
//   {55,15}, // 352
//   {44,15}, // 353
//   {44,15}, // 354
//   {8,1}, // 355
//   {27,15}, // 356
//   {20,15}, // 357
//   {54,15}, // 358
//   {49,15}, // 359
//   {31,0}, // 360
//   {55,3}, // 361
//   {55,15}, // 362
//   {44,16}, // 363
//   {55,15}, // 364
//   {29,15}, // 365
//   {15,15}, // 366
//   {27,15}, // 367
//   {27,15}, // 368
//   {55,3}, // 369
//   {31,13}, // 370
//   {30,15}, // 371
//   {42,1}, // 372
//   {27,15}, // 373
//   {27,15}, // 374
//   {55,15}, // 375
//   {27,15}, // 376
//   {44,15}, // 377
//   {7,15}, // 378
//   {37,15}, // 379
//   {47,15}, // 380
//   {27,15}, // 381
//   {55,15}, // 382
//   {55,15}, // 383
//   {54,15}, // 384
//   {44,15}, // 385
//   {27,1}, // 386
//   {27,1}, // 387
//   {30,0}, // 388
//   {44,15}, // 389
//   {24,15}, // 390
//   {30,1}, // 391
//   {56,15}, // 392
//   {44,15}, // 393
//   {54,13}, // 394
//   {30,9}, // 395
//   {15,15}, // 396
//   {55,10}, // 397
//   {41,15}, // 398
//   {44,15}, // 399
//   {55,15}, // 400
//   {55,18}, // 401
//   {40,15}, // 402
//   {40,4}, // 403
//   {3,13}, // 404
//   {27,15}, // 405
//   {30,4}, // 406
//   {44,14}, // 407
//   {44,1}, // 408
//   {30,15}, // 409
//   {44,1}, // 410
//   {55,15}, // 411
//   {44,3}, // 412
//   {29,14}, // 413
//   {8,15}, // 414
//   {44,15}, // 415
//   {12,15}, // 416
//   {12,15}, // 417
//   {55,15}, // 418
//   {29,0}, // 419
//   {55,13}, // 420
//   {32,1}, // 421
//   {55,15}, // 422
//   {36,15}, // 423
//   {55,13}, // 424
//   {9,18}, // 425
//   {30,15}, // 426
//   {22,9}, // 427
//   {55,15}, // 428
//   {56,15}, // 429
//   {55,1}, // 430
//   {44,1}, // 431
//   {44,0}, // 432
//   {44,0}, // 433
//   {44,15}, // 434
// };

// // ----------------------------------------------------------------------------------------
// // Function to get volcano location by index via volcano_index[0].
// // ----------------------------------------------------------------------------------------
// const char* getVolcanoLocation(int index) {
//     if (index < 0 || index >= MAX_LOCATIONS) {
//         return "undefined";
//     }
//     return volcano_locations[volcano_index[index][0]];
// }

// // ----------------------------------------------------------------------------------------
// // Function to get volcano type by index via volcano_index[1].
// // ----------------------------------------------------------------------------------------
// const char* getVolcanoType(int index) {
//     if (index < 0 || index >= MAX_TYPES) {
//         return "undefined";
//     }
//     return volcano_types[volcano_index[index][1]];
// }

// // ----------------------------------------------------------------------------------------
// // Function to get volcano elevation by index.
// // ----------------------------------------------------------------------------------------
// int getVolcanoElevation(int index) {
//     if (index < 0 || index >= MAX_VOLCANOS) {
//         return 0;
//     }
//     return volcano_elevations[index];
// }

// // ----------------------------------------------------------------------------------------
// // Function to get volcano latitude and longitude by index.
// // ----------------------------------------------------------------------------------------
// void getVolcanoLatLong(int index, float* lat, float* lon) {
//     if (index < 0 || index >= MAX_VOLCANOS || lat == NULL || lon == NULL) {
//         if (lat) *lat = 0.0f;
//         if (lon) *lon = 0.0f;
//         return;
//     }
//     *lat = volcano_lat_long[index][0];
//     *lon = volcano_lat_long[index][1];
// }