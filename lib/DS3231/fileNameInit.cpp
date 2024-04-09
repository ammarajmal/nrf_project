// #include <DS3231.h>

// extern char filename1[];
// extern DS3231 Clock;

// void getFilename() {
    
//     bool Century=false;
//     bool h12, PM;

//     byte cur_month = Clock.getMonth(Century);
//     byte cur_date = Clock.getDate();
//     byte cur_hour = Clock.getHour(h12, PM);
//     byte cur_min = Clock.getMinute();

//     filename1[0]= cur_month / 10 + '0';
//     filename1[1]= cur_month % 10 + '0';
//     filename1[2]= cur_date / 10 + '0';
//     filename1[3]= cur_date % 10 + '0';
//     filename1[4]= cur_hour / 10 + '0';
//     filename1[5]= cur_hour % 10 + '0';
//     filename1[6]= cur_min / 10 + '0';
//     filename1[7]= cur_min % 10 + '0';
// }