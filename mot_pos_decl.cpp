#include <DynamixelWorkbench.h>

#define BAUDRATE  1000000
#define DXL_ID    3

DynamixelWorkbench dxl_wb;

void setup() {
  Serial.begin(57600);
  
  const char *log;
  bool result = false;

  uint8_t dxl_id = DXL_ID;
  uint16_t model_number = 0;

  result = dxl_wb.init(DEVICE_NAME, BAUDRATE, &log);
  if (result == false)
  {
    Serial.println(log);
    Serial.println("Failed to init");
  }
  else
  {
    Serial.print("Succeeded to init : ");
    Serial.println(BAUDRATE);  
  }

  result = dxl_wb.ping(dxl_id, &model_number, &log);
  if (result == false)
  {
    Serial.println(log);
    Serial.println("Failed to ping");
  }
  else
  {
    Serial.println("Succeeded to ping");
    Serial.print("id : ");
    Serial.print(dxl_id);
    Serial.print(" model_number : ");
    Serial.println(model_number);
  }
  
  result = dxl_wb.jointMode(dxl_id, 0, 0, &log);
  if (result == false)
  {
    Serial.println(log);
    Serial.println("Failed to change joint mode");
  }
  else
  {
    Serial.println("Succeed to change joint mode");
  }
  result = dxl_wb.addSyncReadHandler(dxl_id[0], "Present_Position", &log);
  if (result == false)
  {
    Serial.println(log);
    Serial.println("Failed to add sync read handler");
  }
}
void loop()[
  
  const char *log;
  bool result = false;
  
  int32_t present_position = 0;
  
  do
  {
    result = dxl_wb.syncRead(handler_index, &log);
    if (result == false)
    {
      Serial.println(log);
      Serial.println("Failed to sync read position");
    }

    result = dxl_wb.getSyncReadData(handler_index, &present_position[0], &log);
    if (result == false)
    {
      Serial.println(log);
    }
    else
    {
      Serial.print(" Present Position of the steering motor: ");
      Serial.print(present_position);
    }
  }
}
