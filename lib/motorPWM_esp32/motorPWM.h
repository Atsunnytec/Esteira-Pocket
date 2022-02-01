#ifndef MOTORPWM_H
#define MOTORPWM_H

#include <Arduino.h>

class motorPWM
{
public:
    //constructor:
    motorPWM(int Rpwm, int Lpwm, int Ren, int Len)
    {
        _pin_Rpwm = Rpwm;
        _pin_Ren = Ren;
        _pin_Lpwm = Lpwm;
        _pin_Len = Len;
    }

    motorPWM(int Rpwm)
    {
        _pin_Rpwm = Rpwm;
    }

    void setup(long maxSpeed, long rampa) //velocidade de 0 a 100%; rampa in milissegundos
    {
        setMaxSpeed(maxSpeed);
        setRampa(rampa);

        ledcAttachPin(_pin_Rpwm, pwmChannel);               //Atribuímos o pino 2 ao canal 0.
        ledcSetup(pwmChannel, pwmFrequency, pwmResolution); //Atribuímos ao canal 0 a frequencia de 1000Hz com resolucao de 10bits.
        calculaLimiteDaVelocidadeEmBits();
        ledcWrite(pwmChannel, _limiteDaVelocidadeEmBits);
    }

    void calculaLimiteDaVelocidadeEmBits()
    {
        _limiteDaVelocidadeEmBits = 1;
        for (int i = 0; i < pwmResolution; i++)
            _limiteDaVelocidadeEmBits *= 2;
        Serial.print("lim bits: "); Serial.println(_limiteDaVelocidadeEmBits);
    }

    void setRampa(long rampaEmMilissegundos)
    {
        if (rampaEmMilissegundos == 0)
        {
            rampaEmMilissegundos = 1;
        }

        _aceleracao = _velocidade_max / rampaEmMilissegundos;
        calculaDv();
        _rampa = rampaEmMilissegundos; // to do: mover essa linha para cima  e utilizar _rampa nas outras funções.
    }

    void setMaxSpeed(int max) // recebe a velocidade de 0 a 100%
    {
        // converte velocidade para 8bits: 0 a 255
        int temp = constrain(max, 0, cemPorCento);
        _velocidade_max = round((temp * (float)tamanhoUmByte * aumentaResolucao) / cemPorCento);
        Serial.print("velo max: ");
        Serial.println(_velocidade_max);
        setRampa(_rampa);
    }

    void controller()
    {
        if (calculaVelocidade())
        {
            int32_t v = map(_velocidade / 100, 0, tamanhoUmByte, _limiteDaVelocidadeEmBits, 0);
            ledcWrite(pwmChannel, v);
            // Serial.println(v);
        }
    }

    void stop()
    {
        _flag_move = false;
    }

    void run()
    {
        _flag_move = true;
    }

    unsigned long position()
    {
        return _pos / 10000;
    }

    void aceleraEsteira()
    {
        while (_velocidade < _velocidade_max)
        {
            _velocidade += dV;
            int32_t v = map(_velocidade / 100, 0, tamanhoUmByte, _limiteDaVelocidadeEmBits, 0);
            ledcWrite(pwmChannel, v);
            delay(dt);
        }
    }

    void desaceleraEsteira()
    {
        while (_velocidade > 0)
        {
            _velocidade -= dV;
            int32_t v = map(_velocidade / 100, 0, tamanhoUmByte, _limiteDaVelocidadeEmBits, 0);
            ledcWrite(pwmChannel, v);
            delay(dt);
        }
    }

private:
    int _pin_Rpwm;
    int _pin_Ren;
    int _pin_Lpwm;
    int _pin_Len;

    const int16_t pwmChannel = 2;
    const int32_t pwmFrequency = 2500;
    const int16_t pwmResolution = 9;
    int16_t porcento = 50;

    const int16_t tamanhoUmByte = 255;
    const int16_t cemPorCento = 100;
    const int16_t aumentaResolucao = 100;
    long _velocidade_max = 127 * aumentaResolucao; // bits/100 Ex: 5000 => 50bits
    const long _velocidadeMax_emMm = 220;          // mm / s
    long _velocidade_emMm = 0;
    long _pos = 0;
    long _velocidade = 0;  // bits/100
    long _aceleracao = 25; //bits/100ms  Ex: acel = 25 =>>> 0.025%/ms
    long _limiteDaVelocidadeEmBits = 512;
    unsigned int dt = 2; //milissegundos ; período de atualização da velocidade
    unsigned int dV = dt * _aceleracao;
    long _rampa = 1000;

    bool _flag_move = false;
    unsigned long timer_dt = 0;

    bool calculaVelocidade()
    {
        if (millis() - timer_dt >= dt)
        {
            timer_dt = millis();
            if (_flag_move == true) // acelera
            {
                if (_velocidade < _velocidade_max)
                {
                    _velocidade += dV;
                    _velocidade = constrain(_velocidade, 0, _velocidade_max);
                    _velocidade_emMm = _velocidadeMax_emMm * 10 / 100 * _velocidade / tamanhoUmByte;
                    return true;
                }
            }
            else if (_flag_move == false) // desacelera
            {
                if (_velocidade > 0)
                {
                    _velocidade -= dV;
                    _velocidade = constrain(_velocidade, 0, _velocidade_max);
                    _velocidade_emMm = _velocidadeMax_emMm * 10 / 100 * _velocidade / tamanhoUmByte;
                    return true;
                }
            }

            unsigned int ds = _velocidade_emMm * dt;
            _pos += ds;
        }
        return false;
    }

    void calculaDv()
    {
        dV = dt * _aceleracao;
    }
};
#endif