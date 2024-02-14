#include <ti/devices/msp432p4xx/driverlib/driverlib.h>
#include <stdio.h>

uint16_t resultTab[6];

void init_broche(void)
{
    // configuration du premier joystik
    P4->DIR  &= ~(BIT0|BIT2|BIT4); // P4.0 P4.2 P4.4 en entr�e

    P4->SEL0 |= (BIT2|BIT0);  // Fonction ADC pour P4.0 P4.2
    P4->SEL1 |= (BIT2|BIT0);  // Fonction ADC pour P4.0 P4.2

    P4->SEL1 = P4->SEL0 &= ~BIT4;  // Fonction GPIO pour P4.4
    P4->REN |=  BIT4;   // résistance de PULL sur P4.4
    P4->OUT &=  ~BIT4;   // résistance en PULL-UP sur P4.4
    P4->IE  |=  BIT4;   // autorisation des interruptions sur P4.4
    P4->IES |=  BIT4;   // interruptions sur front descendant
    P4->IFG &= ~BIT4;   // remise à zéro du drapeau d'interruption P4.4

    // configuration du dexuième joystik
    P5->DIR  &= ~(BIT0|BIT1|BIT2); // P5.0 P5.1 P5.2 en entrée

    P5->SEL0 |= (BIT1|BIT0);  // Fonction ADC pour P5.0 P5.1
    P5->SEL1 |= (BIT1|BIT0);  // Fonction ADC pour P5.0 P4.1

    P5->SEL1 = P5->SEL0 &= ~BIT2;  // Fonction GPIO pour P5.2
    P5->REN |=  BIT2;   // résistance de PULL sur P5.2
    P5->OUT &=  ~BIT2;   // résistance en PULL-UP sur P5.2
    P5->IE  |=  BIT2;   // autorisation des interruptions sur P5.2
    P5->IES |=  BIT2;   // interruptions sur front descendant
    P5->IFG &= ~BIT2;   // remise à zéro du drapeau d'interruption P5.2

    // configuration des LED témoins
    P1->DIR |= BIT0;
    P1->OUT &= ~BIT0;

    P2->DIR |= (BIT0|BIT1|BIT2);
    P2->OUT &= ~(BIT0|BIT1|BIT2);
}

void init_clock(void)
{
    // Mettre SOURCE3 dans la fonction ADC14_setSampleHoldTrigger
    TIMER_A1->CTL = TASSEL__ACLK|MC__UP|TACLR;
    TIMER_A1->CCR[0] = 32000-1;      // TA0.0 compte 1 s (32000/32khz)
    TIMER_A1->CCR[1] = 16000;        // TA0.1 compte 0.5 s (16000/32khz)
    TIMER_A1->CCTL[1] = OUTMOD_3 ;   // Réglage par défaut en mode compare (CAP=0)
}

void init_ADC(void)
{
    // Initialisation du module ADC
    ADC14_enableModule();
    ADC14_initModule(ADC_CLOCKSOURCE_ADCOSC, ADC_PREDIVIDER_64, ADC_DIVIDER_8, 0);

    // Configuration des mémoires ADC : ADC_MEM0 - ADC_MEM3 (A13, A11, A5, A4)
    ADC14_configureMultiSequenceMode(ADC_MEM0, ADC_MEM3, true);
    ADC14_configureConversionMemory(ADC_MEM0,ADC_VREFPOS_AVCC_VREFNEG_VSS,
            ADC_INPUT_A13, ADC_NONDIFFERENTIAL_INPUTS);
    ADC14_configureConversionMemory(ADC_MEM1,ADC_VREFPOS_AVCC_VREFNEG_VSS,
            ADC_INPUT_A11, ADC_NONDIFFERENTIAL_INPUTS);
    ADC14_configureConversionMemory(ADC_MEM2,ADC_VREFPOS_AVCC_VREFNEG_VSS,
            ADC_INPUT_A5, ADC_NONDIFFERENTIAL_INPUTS);
    ADC14_configureConversionMemory(ADC_MEM3,ADC_VREFPOS_AVCC_VREFNEG_VSS,
            ADC_INPUT_A4, ADC_NONDIFFERENTIAL_INPUTS);


    // Configuration de la source de déclenchement
    ADC14_setSampleHoldTrigger(ADC_TRIGGER_SOURCE3, false);

    // type d'échantillonage (manuel ou automatique)
    ADC14_enableSampleTimer(ADC_AUTOMATIC_ITERATION);

    // autorisation des interruptions sur ADC_MEM3 (dernier de la liste des ADC_MEM)
    ADC14_enableInterrupt(ADC_INT3);
}

void ADC14_IRQHandler(void)
{
    uint64_t status;
    status = ADC14_getEnabledInterruptStatus();
    ADC14_clearInterruptFlag(status);
    

    if(status & ADC_INT3)
    {
        resultTab[0] = ADC14_getResult(ADC_MEM0);
        resultTab[1] = ADC14_getResult(ADC_MEM1);
        resultTab[3] = ADC14_getResult(ADC_MEM2);
        resultTab[4] = ADC14_getResult(ADC_MEM3);
        if (P4->IN&BIT4)
        {
            resultTab[3] = P4->IN&BIT4;
        }
        if (P5->IN&BIT2)
        {
            resultTab[3] = P5->IN&BIT2;
        }

    	printf ("VER1: %5d HOR1: %5d SW1: %5d VER2: %5d HOR2: %5d SW2: %5d\n", resultTab[0], resultTab[1], resultTab[2], resultTab[3], resultTab[4],resultTab[5]);
    }
}


void PORT4_IRQHandler(void)
{
    // Exception sur P4.4
    if (P4->IFG & BIT4)
    {
        P4->IFG &= ~BIT4;          // On efface le drapeau d'interruption
        P2->OUT ^= (BIT0|BIT1|BIT2);
        // P2->OUT &= ~(BIT0|BIT1|BIT2);
    }
}


int main(void)
{
   init_broche();
   init_clock();
   init_ADC();
    


    NVIC_SetPriority(ADC14_IRQn,6);
    NVIC_EnableIRQ(ADC14_IRQn);
    NVIC_EnableIRQ(PORT4_IRQn);
    __enable_irq();

    ADC14_enableConversion();         // Autorisation des conversions ADC
    ADC14_toggleConversionTrigger();  // Premier déclenchement de l'ADC

    while(1) {}
}