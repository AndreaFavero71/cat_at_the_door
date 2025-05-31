"""
Andrea Favero, 20250603 

'Door at the door' project

Convert Unix timestamp to a tuple of date and time fields (year, month, day, hours, minutes, seconds).
Adjust for UTC shift and DST shift (if applicable).

"""


import utime

class DateTimeConverter:
    def __init__(self):
        pass

    def unix_to_datetime(self, unix_time, utc_shift=0, dst_shift=0):
        """
        Convert Unix timestamp to a tuple (year, month, day, hours, minutes, seconds),
        adjusted for UTC shift and DST shift (if applicable).
        """
        # Adjust for MicroPython's epoch (2000-01-01)
#         unix_time += 946684800

        # Apply UTC and DST shifts
        if self.is_summer_time(unix_time):
            unix_time += (utc_shift + dst_shift) * 3600
        else:
            unix_time += utc_shift * 3600

        # Convert Unix time to a time tuple
        time_tuple = utime.localtime(unix_time)

        return time_tuple[:6]  # Return (year, month, day, hours, minutes, seconds)

    
    def is_summer_time(self, unix_time):
        """
        Check if the given Unix timestamp falls within the DST period
        (last Sunday of March to last Saturday of October).
        """
        # Convert Unix time to a time tuple
        time_tuple = utime.localtime(unix_time)
        year, month, day = time_tuple[0], time_tuple[1], time_tuple[2]

        if month > 3 and month < 10:
            return True  # Definitely in summer time
        elif month == 3:
            last_sunday_march = self.get_last_sunday(year, 3)
            return day >= last_sunday_march  # On or after last Sunday of March
        elif month == 10:
            last_sunday_october = self.get_last_sunday(year, 10)
            return day < last_sunday_october  # Before last Sunday of October
        else:
            return False  # Not in summer time

    
    def get_last_sunday(self, year, month):
        """
        Get the last Sunday of a given month and year.
        """
        if month == 3:      # March
            last_day = 31
        elif month == 10:   # October
            last_day = 31
        else:
            return None  # Only March and October are relevant for DST

        # Find the last Sunday
        for day in range(last_day, 0, -1):
            # Use Zeller's congruence to find the day of the week
            if month < 3:
                month += 12
                year -= 1
            q = day
            m = month
            K = year % 100
            J = year // 100
            h = (q + (13 * (m + 1)) // 5 + K + (K // 4) + (J // 4) + 5 * J) % 7
            # h = 0 corresponds to Saturday, 1 = Sunday, 2 = Monday, etc.
            if h == 1:  # Sunday
                return day
        return None


# Example usage
if __name__ == "__main__":
    converter = DateTimeConverter()
    now = utime.time()
    year, month, day, hours, minutes, seconds = converter.unix_to_datetime(now, utc_shift=1, dst_shift=1)
    date_now = f"{year}-{month:02d}-{day:02d}"
    time_now = f"{hours:02d}:{minutes:02d}:{seconds:02d}"
    print(f"Date: {date_now}")
    print(f"Time: {time_now}")