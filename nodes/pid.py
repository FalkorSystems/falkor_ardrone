#!/usr/bin/env python

class Pid2:
    def __init__( self, gain_p = 0.0, gain_i = 0.0, gain_d = 0.0,
                  time_constant = 0.0, limit = -1.0 ):
        self.gain_p = gain_p
        self.gain_i = gain_i
        self.gain_d = gain_d
        self.time_constant = time_constant
        self.limit = limit

        self.input = 0.0
        self.dinput = 0.0
        self.output = 0.0
        self.p = 0.0
        self.i = 0.0
        self.d = 0.0

    def update( self, new_input, x, dx, dt ):
        if self.limit > 0.0 and abs( new_input ) > self.limit:
            if new_input < 0:
                new_input = - self.limit
            else:
                new_input = self.limit

        if dt + self.time_constant > 0.0:
            self.dinput = ( new_input - self.input ) / ( dt + self.time_constant )
            self.input = ( dt * new_input + self.time_constant * self.input ) / ( dt + self.time_constant )

        self.p = self.input - x
        self.d = self.dinput - dx
        self.i = self.i + dt * self.p

#        print self.p,self.d,self.i

        self.output = self.gain_p * self.p + self.gain_d * self.d + self.gain_i * self.i
        return self.output

    def reset( self ):
        self.input = self.dinput = 0.0
        self.p = self.i = self.d = 0.0

        
class Pid:
    def __init__( self, Kp = 0.0, Ti = 0.0, Td = 0.0, outputLimit = None ):
        self.Kp = Kp
        self.Ti = Ti
        self.Td = Td

        self.prev_error = 0.0
        self.integral = 0.0

        self.outputLimit = outputLimit
        self.setPointMin = self.setPointMax = 0.0

    def get_output( self, pv, dt ):
        if( pv < self.setPointMax and pv > self.setPointMin ):
            error = 0
        else:
            errorFromMax = self.setPointMax - pv
            errorFromMin = self.setPointMin - pv

            if abs( errorFromMax ) > abs( errorFromMin ):
                error = errorFromMin
            else:
                error = errorFromMax

        self.integral += error * dt

        if dt != 0:
            derivative = ( error - self.prev_error ) / dt
        else:
            derivative = 0

        self.prev_error = error

        if self.Ti == 0.0:
            TiInv = 0
        else:
            TiInv = 1/self.Ti

        output = self.Kp * ( error + TiInv * self.integral + self.Td * derivative )
        if self.outputLimit != None:
            limitedOutput = min( max( output, -self.outputLimit ),
                                 self.outputLimit )
            return limitedOutput
        else:
            return output
