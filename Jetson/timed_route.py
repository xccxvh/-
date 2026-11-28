"""Measured O -> +5 -> -5 cm route on the glued ruler. Positive x needs lower PWM."""
class TimedRoute:
    def __init__(self, low, neutral, high):
        self.low, self.neutral, self.high = low, neutral, high
        self.goal = 5.
        self.first_hit = None
        self.hold_start = None
        self.endpoint = None
        self.bias = 0.
        self.last_stamp = None

    def check_endpoint(self, target, output, exposure_output, stamp, x, v):
        # Same stationary-on-applied-limit evidence as the O-point controller.
        if target not in (self.low,self.high) or output != target or exposure_output != target or abs(v)>=.25:
            self.endpoint = None
        elif self.endpoint is None or self.endpoint[0]!=target or abs(x-self.endpoint[2])>=.06:
            self.endpoint = (target, stamp, x)
        elif stamp-self.endpoint[1]>=.35:
            raise ValueError('mechanical endpoint without measured motion')

    def interrupt_hold(self):
        self.endpoint = None
        self.hold_start = None
        self.last_stamp = None

    def step(self, x, v, stamp, measured):
        dt = stamp-self.last_stamp if self.last_stamp is not None else 0.
        self.last_stamp = stamp
        turned = self.goal > 0 and 4.3 <= measured <= 5.7
        if turned:
            self.first_hit = stamp
            self.goal = -5.
            self.bias = 0.
        error = self.goal-x
        # Starting tuning: a 5 cm stationary error requests about 60 ticks;
        # prior field data showed 50 ticks could break static friction.
        effort = 12.*error - 15.*v + self.bias
        if abs(v)<.4 and abs(error)>.35 and self.low < self.neutral-effort < self.high:
            self.bias += 3.*error*dt
        settled = self.goal < 0 and abs(measured+5.) <= 1. and abs(v)<=.35
        if settled:
            if self.hold_start is None:
                self.hold_start=stamp
        else:
            self.hold_start=None
        return dict(target_ticks=round(max(self.low,min(self.high,self.neutral-effort))),
                    phase='endpoint_hold' if settled else 'route_return' if self.goal<0 else 'route_out',
                    reason='measured positive target band' if turned else 'position and velocity feedback',
                    hold_s=stamp-self.hold_start if settled else 0.)
