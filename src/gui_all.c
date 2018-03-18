#include "gui_all.h"
#include "cmsis_os.h"
#include <GUI.h>
#include "WM.h"
#include "FRAMEWIN.h"
/*#include "makise.h"
#include "makise_gui.h"
#include "makise_e.h"
#include "LCDConf.h"
#include "GLCD.h"*/
#include "hid_proc.h"

static FRAMEWIN_Handle _ahFrameWin[2];

/*MakiseGUI* mGui;
MHost *host;

MakiseGUI    Gu;
MakiseBuffer Bu;
MakiseDriver Dr;
MHost hs;
MContainer co;

MPosition ma_g_hpo;

MakiseGUI    * gu;
MakiseBuffer * bu;
MakiseDriver * dr;

uint8_t makise_LCD_init (MakiseGUI* gui)
{
	LCD_Initializtion();
	return M_OK;
}
uint8_t makise_LCD_start(MakiseGUI* gui)
{
    return M_OK;
}
uint8_t makise_LCD_sleep(MakiseGUI* gui)
{
    return M_OK;
}
uint8_t makise_LCD_awake(MakiseGUI* gui)
{
    return M_OK;
}
uint8_t makise_LCD_set_backlight(MakiseGUI* gui, uint8_t b)
{
    return M_OK;
}

void makise_LCD_draw(MakiseGUI* gui)
{ 
	MakiseDriver * d = gui->driver;
	uint16_t c;

	if(d->gui->draw != 0)
		d->gui->draw(d->gui);

	uint32_t y = 0, x;

	for (; y < d->lcd_height; ++y) 
	{	
		for (x = 0; x < d->lcd_width; ++x)
		{
			c = makise_pget_fast(gui->buffer, x, y);
			LCD_SetPoint(x, y, c);
		}
	}
	memset(gui->buffer->buffer, 0, 960000);


	if(d->gui->predraw != 0)
		d->gui->predraw(d->gui);
	if(d->gui->update != 0)
		d->gui->update(d->gui);
}*/

void init_GUI(void)
{
	GUI_Init();
	/*LCD_Initializtion();
	gu = &Gu;
	bu = &Bu;
	dr = &Dr;
	
	host = &hs;
	host->host = &co;
	host->host->gui = gu;
	makise_gui_init(host); //init gui host
	//if input event wasn't handled by gui. We need to handle it
	//host->input.result_handler = &inp_handler;

	ma_g_hpo = mp_rel(0,0,320,240);
	ma_g_hpo.real_x = 0;
	ma_g_hpo.real_y = 0;
	host->host->position = &ma_g_hpo;
	
	Dr.lcd_height = LCD_YSIZE;
	Dr.lcd_width = LCD_XSIZE;
	Dr.buffer_height = MAKISE_BUF_H;
  Dr.buffer_width  = MAKISE_BUF_W;
	Dr.pixeldepth    = 16;
	Dr.buffer        = 0;
	Dr.size          = 0;
	Dr.posx          = 0;
	Dr.posy          = 0;
	Dr.init          = &makise_LCD_init;
	Dr.start         = &makise_LCD_start;
	Dr.sleep         = &makise_LCD_sleep;
	Dr.awake         = &makise_LCD_awake;
	Dr.set_backlight = &makise_LCD_set_backlight;

	//init driver structure
	//makise_sdl2_driver(dr, 320, 240, screen);

	uint32_t sz = makise_init(gu, dr, bu);
	
	mGui = gu;
	makise_start(gu);

	mGui->predraw = 0; //we don't need driver to execute those methods
	mGui->draw = 0;*/
}

/*static MTextField textfield;
static char about[] = "This sample shows how to create buttons and handle their events. ";

MakiseStyle ts_textfield =
{
    MC_White,
    &F_Arial15,
    3,
    //bg       font     border   double_border
    {MC_Black, MC_White, MC_Gray, 0},  //unactive
    {MC_Black, MC_White, MC_Green, 0},  //unactive
    {0, 0, 0, 0}, //focused
    {0, 0, 0, 0}, //active
};*/

void main_GUI(void)
{
	int xCenter = LCD_GET_XSIZE() / 2;
  int y;
	
	/*m_create_text_field(&textfield,  //pointer to the structure
			host->host, //container
			mp_rel(0, 0,    //position
			       320, 50), //width, height
			about,    //text
			&ts_textfield//style
	);*/
		
	GUI_SetBkColor(GUI_BLUE);
	GUI_SetColor(GUI_LIGHTRED);
	GUI_Clear();
  osDelay(10);
	char buf[256];
	
	while (1)
	{
		/*makise_g_host_call(host, M_G_CALL_PREDRAW);
		makise_g_host_call(host, M_G_CALL_DRAW);
		
		makise_render(mGui, 0);
		
		makise_LCD_draw(mGui);*/
		
		GUI_SetFont(&GUI_Font24_ASCII);
		GUI_DispStringAt("HID scancodes: ", 5, 30);
		sprintf(buf, "%04X %04X %04X %04X\n  %04X %04X %04X %04X", keys_pressed[0], keys_pressed[1], 
				keys_pressed[2], keys_pressed[3], keys_pressed[4], keys_pressed[5], keys_pressed[6], keys_pressed[7]);
		GUI_DispStringAt(buf, 15, 55);
		
		WM_ExecIdle();
		
    osDelay(10);
	}
}
