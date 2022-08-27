//#ifdef _WDG_Supervision

int Supervision_init(int Supervision_state)
{
    
}

void interrupt menu(void)
{
    if (INTCONbits.INTF == 1)
    {
        INTCONbits.INTF = 0;
        
    }
}






















//#endif