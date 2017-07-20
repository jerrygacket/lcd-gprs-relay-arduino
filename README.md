# lcd-gprs-relay-arduino
lcd keypad gprs sim900 relay arduino

Управление двумя реле с контролем их включения и включения, контролем аналогового сигнала датчика уровня жидкости. Вывод этих данных на модуль lcd keypad 16x2. Отправка данных на срвер через GPRS с помощью модуля SIM900. Включение сирены в случае критически низкого уровня жидкости.

В модулях srd-05vdc-sl-c зеленый светодиод (контроль подачи управляющего сигнала) заменен на опттопару PS2501. 3 контакт оптопары подключен к земле, 4 - к входу ардуино для контроля работы реле.

В следующей версии устройства необходимо подключить оптопару к линии питания нагрузки.

Компоненты:

 -- Arduino uno R3
 
 -- GSM/GPRS модем SIM900 Shield
 
 -- LCD1602 Keypad Shield
 
 -- 2 штуки Arduino Relay module srd-05vdc-sl-c
 
 -- 2 оптопары (PS2501)
 
 -- пъезо пищалка
