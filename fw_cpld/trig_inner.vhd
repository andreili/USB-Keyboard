library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity trig_inner is
	port
	(
		sig_to_tr	: in	std_logic;
		a16			: in	std_logic;
		tin			: in	std_logic;
		vector		: in	std_logic_vector(7 downto 0);
		data			: in	std_logic_vector(7 downto 0);
		odata			: out	std_logic_vector(7 downto 0)
	);
end entity;

architecture rtl of trig_inner is

signal kjoy1			: std_logic_vector(4 downto 0);
signal kjoy2			: std_logic_vector(4 downto 0);
signal fjoy				: std_logic_vector(4 downto 0);
signal zxkbd0			: std_logic_vector(4 downto 0);
signal zxkbd1			: std_logic_vector(4 downto 0);
signal zxkbd2			: std_logic_vector(4 downto 0);
signal zxkbd3			: std_logic_vector(4 downto 0);
signal zxkbd4			: std_logic_vector(4 downto 0);
signal zxkbd5			: std_logic_vector(4 downto 0);
signal zxkbd6			: std_logic_vector(4 downto 0);
signal zxkbd7			: std_logic_vector(4 downto 0);

begin

process (sig_to_tr)
begin
	if (rising_edge(sig_to_tr)) then -- end of cycle
		case (vector) is
			when x"01" =>	kjoy1  <= data(4 downto 0);
			when x"02" =>	fjoy   <= data(4 downto 0);
			when x"03" =>	kjoy2  <= data(4 downto 0);
			when x"04" =>	zxkbd0 <= data(4 downto 0);
			when x"05" =>	zxkbd1 <= data(4 downto 0);
			when x"06" =>	zxkbd2 <= data(4 downto 0);
			when x"07" =>	zxkbd3 <= data(4 downto 0);
			when x"08" =>	zxkbd4 <= data(4 downto 0);
			when x"09" =>	zxkbd5 <= data(4 downto 0);
			when x"0a" =>	zxkbd6 <= data(4 downto 0);
			when x"0b" =>	zxkbd7 <= data(4 downto 0);
			when others =>	fjoy <= (others => 'Z');
		end case;
	end if;
end process;

odata <= "000" & kjoy1 when ((a16 = '1') and (vector = x"1f")) else
			--fjoy(4) & "111" & fjoy(3 downto 0) when ((a16 = '1') and (vector = x"7f")) else
			"000" & kjoy2 when ((a16 = '1') and (vector = x"df")) else
			'0' & tin & '0' & zxkbd0 when (vector = x"04") else
			'0' & tin & '0' & zxkbd1 when (vector = x"05") else
			'0' & tin & '0' & zxkbd2 when (vector = x"06") else
			'0' & tin & '0' & zxkbd3 when (vector = x"07") else
			'0' & tin & '0' & zxkbd4 when (vector = x"7f") else
			'0' & tin & '0' & zxkbd5 when (vector = x"bf") else
			'0' & tin & '0' & zxkbd6 when (vector = x"df") else
			'0' & tin & '0' & zxkbd7 when (vector = x"ef") else
			x"30"			   when (vector = x"0f") else
			(others => 'Z');

end rtl;
