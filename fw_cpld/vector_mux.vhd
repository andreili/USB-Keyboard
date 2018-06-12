library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity vector_mux is
	port
	(
		addr	 		: in	std_logic_vector(17 downto 0);
		vector		: out	std_logic_vector( 7 downto 0);
		int_to_stm	: out	std_logic
	);
end entity;

architecture rtl of vector_mux is

-- vector decoding
signal mux_lo			: std_logic;
signal mux_hi			: std_logic;
signal vector_int		: std_logic_vector(7 downto 0);

begin

mux_lo <= (addr(15) and addr(3));
vector_int(3 downto 0) <= addr(7 downto 4) when (mux_lo= '1')		-- L
							else addr(3 downto 0);								-- l
mux_hi <= (addr(15) and addr(3)) or addr(7);
vector_int(7 downto 4) <= addr(7 downto 4) when (mux_hi = '0')		-- h
							else addr(15 downto 12) when (mux_lo = '0')	-- H
							else addr(11 downto 8);								-- L

int_to_stm <=	'1' when (	(vector_int = x"1f") or
									(vector_int = x"57") or
									(vector_int = x"77") or
									(vector_int = x"7f") or
									(vector_int = x"df") or
									(vector_int = x"0f") or
									((vector_int(3 downto 0) = x"d") and (vector_int(7) = '0')) or
									(vector_int = x"b7") or
									(vector_int = x"d7") or
									(vector_int = x"e7") or
									((vector_int(3 downto 0) = x"e") and (vector_int(7) = '1')) or
									(vector_int = x"ad") or
									(vector_int = x"bd") or
									(vector_int = x"fd"))
					else '0';

vector <= vector_int;

end rtl;
