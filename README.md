# esp32_bme280_test

New_user, First use of the Esp32 + "Bosh's" bme280 temp-pressure-humidity sensor 

# Usage
git clone --recursive https://github.com/ScottyGitAccount2/esp32_bme280_test.git

# Issue. & Fix. 
missing bme280_driver CMakeList.txt..
create a .txt document, saved as CMakeList.txt with the following code:

**set(bme_srcs "bme280.c")**

**idf_component_register(SRCS "${bme_srcs}"**
  **INCLUDE_DIRS  ".")**

Place the document in the folder/directory:  
**esp32_bme280_test/components/BME280_driver**


