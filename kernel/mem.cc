/*
 *  yosys -- Yosys Open SYnthesis Suite
 *
 *  Copyright (C) 2020  Marcelina Kościelnicka <mwk@0x04.net>
 *
 *  Permission to use, copy, modify, and/or distribute this software for any
 *  purpose with or without fee is hereby granted, provided that the above
 *  copyright notice and this permission notice appear in all copies.
 *
 *  THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 *  WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 *  MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 *  ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 *  WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 *  ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 *  OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 *
 */

#include "kernel/mem.h"
#include "kernel/ff.h"

USING_YOSYS_NAMESPACE

void Mem::remove() {
	if (cell) {
		module->remove(cell);
		cell = nullptr;
	}
	if (mem) {
		module->memories.erase(mem->name);
		delete mem;
		mem = nullptr;
	}
	for (auto &port : rd_ports) {
		if (port.cell) {
			module->remove(port.cell);
			port.cell = nullptr;
		}
	}
	for (auto &port : wr_ports) {
		if (port.cell) {
			module->remove(port.cell);
			port.cell = nullptr;
		}
	}
	for (auto &init : inits) {
		if (init.cell) {
			module->remove(init.cell);
			init.cell = nullptr;
		}
	}
}

void Mem::emit() {
	if (packed) {
		if (mem) {
			module->memories.erase(mem->name);
			delete mem;
			mem = nullptr;
		}
		if (!cell) {
			if (memid.empty())
				memid = NEW_ID;
			cell = module->addCell(memid, ID($mem));
		}
		cell->attributes = attributes;
		cell->parameters[ID::MEMID] = Const(memid.str());
		cell->parameters[ID::WIDTH] = Const(width);
		cell->parameters[ID::OFFSET] = Const(start_offset);
		cell->parameters[ID::SIZE] = Const(size);
		cell->parameters[ID::RD_PORTS] = Const(GetSize(rd_ports));
		cell->parameters[ID::WR_PORTS] = Const(GetSize(wr_ports));
		Const rd_clk_enable, rd_clk_polarity, rd_transparency_mask;
		Const wr_clk_enable, wr_clk_polarity, wr_priority_mask;
		Const rd_ce_over_srst, rd_arst_value, rd_srst_value, rd_init_value;
		SigSpec rd_clk, rd_en, rd_addr, rd_data;
		SigSpec rd_arst, rd_srst;
		SigSpec wr_clk, wr_en, wr_addr, wr_data;
		int abits = 0;
		for (auto &port : rd_ports)
			abits = std::max(abits, GetSize(port.addr));
		for (auto &port : wr_ports)
			abits = std::max(abits, GetSize(port.addr));
		cell->parameters[ID::ABITS] = Const(abits);
		for (auto &port : rd_ports) {
			if (port.cell) {
				module->remove(port.cell);
				port.cell = nullptr;
			}
			rd_clk_enable.bits.push_back(State(port.clk_enable));
			rd_clk_polarity.bits.push_back(State(port.clk_polarity));
			log_assert(GetSize(port.transparency_mask) == GetSize(wr_ports));
			for (bool bit : port.transparency_mask)
				rd_transparency_mask.bits.push_back(State(bit));
			rd_ce_over_srst.bits.push_back(State(port.ce_over_srst));
			log_assert(GetSize(port.arst_value) == width);
			for (auto &bit : port.arst_value)
				rd_arst_value.bits.push_back(bit);
			log_assert(GetSize(port.srst_value) == width);
			for (auto &bit : port.srst_value)
				rd_srst_value.bits.push_back(bit);
			log_assert(GetSize(port.init_value) == width);
			for (auto &bit : port.init_value)
				rd_init_value.bits.push_back(bit);
			rd_clk.append(port.clk);
			log_assert(GetSize(port.clk) == 1);
			rd_arst.append(port.arst);
			log_assert(GetSize(port.arst) == 1);
			rd_srst.append(port.srst);
			log_assert(GetSize(port.srst) == 1);
			rd_en.append(port.en);
			log_assert(GetSize(port.en) == 1);
			SigSpec addr = port.addr;
			addr.extend_u0(abits, false);
			rd_addr.append(addr);
			log_assert(GetSize(addr) == abits);
			rd_data.append(port.data);
			log_assert(GetSize(port.data) == width);
		}
		if (rd_ports.empty()) {
			rd_clk_enable = State::S0;
			rd_clk_polarity = State::S0;
			rd_transparency_mask = State::S0;
			rd_ce_over_srst = State::S0;
			rd_arst_value = State::S0;
			rd_srst_value = State::S0;
			rd_init_value = State::S0;
		}
		if (wr_ports.empty()) {
			rd_transparency_mask = State::S0;
		}
		cell->parameters[ID::RD_CLK_ENABLE] = rd_clk_enable;
		cell->parameters[ID::RD_CLK_POLARITY] = rd_clk_polarity;
		cell->parameters[ID::RD_TRANSPARENCY_MASK] = rd_transparency_mask;
		cell->parameters[ID::RD_CE_OVER_SRST] = rd_ce_over_srst;
		cell->parameters[ID::RD_ARST_VALUE] = rd_arst_value;
		cell->parameters[ID::RD_SRST_VALUE] = rd_srst_value;
		cell->parameters[ID::RD_INIT_VALUE] = rd_init_value;
		cell->setPort(ID::RD_CLK, rd_clk);
		cell->setPort(ID::RD_EN, rd_en);
		cell->setPort(ID::RD_ARST, rd_arst);
		cell->setPort(ID::RD_SRST, rd_srst);
		cell->setPort(ID::RD_ADDR, rd_addr);
		cell->setPort(ID::RD_DATA, rd_data);
		for (auto &port : wr_ports) {
			if (port.cell) {
				module->remove(port.cell);
				port.cell = nullptr;
			}
			wr_clk_enable.bits.push_back(State(port.clk_enable));
			wr_clk_polarity.bits.push_back(State(port.clk_polarity));
			log_assert(GetSize(port.priority_mask) == GetSize(wr_ports));
			for (bool bit : port.priority_mask)
				wr_priority_mask.bits.push_back(State(bit));
			wr_clk.append(port.clk);
			log_assert(GetSize(port.clk) == 1);
			wr_en.append(port.en);
			log_assert(GetSize(port.en) == width);
			SigSpec addr = port.addr;
			addr.extend_u0(abits, false);
			wr_addr.append(addr);
			log_assert(GetSize(addr) == abits);
			wr_data.append(port.data);
			log_assert(GetSize(port.data) == width);
		}
		if (wr_ports.empty()) {
			wr_clk_enable = State::S0;
			wr_clk_polarity = State::S0;
			wr_priority_mask = State::S0;
		}
		cell->parameters[ID::WR_CLK_ENABLE] = wr_clk_enable;
		cell->parameters[ID::WR_CLK_POLARITY] = wr_clk_polarity;
		cell->parameters[ID::WR_PRIORITY_MASK] = wr_priority_mask;
		cell->setPort(ID::WR_CLK, wr_clk);
		cell->setPort(ID::WR_EN, wr_en);
		cell->setPort(ID::WR_ADDR, wr_addr);
		cell->setPort(ID::WR_DATA, wr_data);
		for (auto &init : inits) {
			if (init.cell) {
				module->remove(init.cell);
				init.cell = nullptr;
			}
		}
		cell->parameters[ID::INIT] = get_init_data();
	} else {
		if (cell) {
			module->remove(cell);
			cell = nullptr;
		}
		if (!mem) {
			if (memid.empty())
				memid = NEW_ID;
			mem = new RTLIL::Memory;
			mem->name = memid;
			module->memories[memid] = mem;
		}
		mem->width = width;
		mem->start_offset = start_offset;
		mem->size = size;
		for (auto &port : rd_ports) {
			if (!port.cell)
				port.cell = module->addCell(NEW_ID, ID($memrd));
			port.cell->parameters[ID::MEMID] = memid.str();
			port.cell->parameters[ID::ABITS] = GetSize(port.addr);
			port.cell->parameters[ID::WIDTH] = width;
			port.cell->parameters[ID::CLK_ENABLE] = port.clk_enable;
			port.cell->parameters[ID::CLK_POLARITY] = port.clk_polarity;
			port.cell->parameters[ID::TRANSPARENCY_MASK] = port.transparency_mask;
			port.cell->parameters[ID::CE_OVER_SRST] = port.ce_over_srst;
			port.cell->parameters[ID::ARST_VALUE] = port.arst_value;
			port.cell->parameters[ID::SRST_VALUE] = port.srst_value;
			port.cell->parameters[ID::INIT_VALUE] = port.init_value;
			port.cell->setPort(ID::CLK, port.clk);
			port.cell->setPort(ID::EN, port.en);
			port.cell->setPort(ID::ARST, port.arst);
			port.cell->setPort(ID::SRST, port.srst);
			port.cell->setPort(ID::ADDR, port.addr);
			port.cell->setPort(ID::DATA, port.data);
		}
		int idx = 0;
		for (auto &port : wr_ports) {
			if (!port.cell)
				port.cell = module->addCell(NEW_ID, ID($memwr));
			port.cell->parameters[ID::MEMID] = memid.str();
			port.cell->parameters[ID::ABITS] = GetSize(port.addr);
			port.cell->parameters[ID::WIDTH] = width;
			port.cell->parameters[ID::CLK_ENABLE] = port.clk_enable;
			port.cell->parameters[ID::CLK_POLARITY] = port.clk_polarity;
			port.cell->parameters[ID::PORTID] = idx++;
			port.cell->parameters[ID::PRIORITY_MASK] = port.priority_mask;
			port.cell->setPort(ID::CLK, port.clk);
			port.cell->setPort(ID::EN, port.en);
			port.cell->setPort(ID::ADDR, port.addr);
			port.cell->setPort(ID::DATA, port.data);
		}
		idx = 0;
		for (auto &init : inits) {
			if (!init.cell)
				init.cell = module->addCell(NEW_ID, ID($meminit));
			init.cell->parameters[ID::MEMID] = memid.str();
			init.cell->parameters[ID::ABITS] = GetSize(init.addr);
			init.cell->parameters[ID::WIDTH] = width;
			init.cell->parameters[ID::WORDS] = GetSize(init.data) / width;
			init.cell->parameters[ID::PRIORITY] = idx++;
			init.cell->setPort(ID::ADDR, init.addr);
			init.cell->setPort(ID::DATA, init.data);
		}
	}
}

void Mem::remove_wr_port(int idx) {
	if (wr_ports[idx].cell) {
		module->remove(wr_ports[idx].cell);
	}
	wr_ports.erase(wr_ports.begin() + idx);
	for (auto &port: wr_ports) {
		port.priority_mask.erase(port.priority_mask.begin() + idx);
	}
	for (auto &port: rd_ports) {
		port.transparency_mask.erase(port.transparency_mask.begin() + idx);
	}
}

void Mem::remove_rd_port(int idx) {
	if (rd_ports[idx].cell) {
		module->remove(rd_ports[idx].cell);
	}
	rd_ports.erase(rd_ports.begin() + idx);
}

void Mem::clear_inits() {
	for (auto &init : inits)
		if (init.cell)
			module->remove(init.cell);
	inits.clear();
}

Const Mem::get_init_data() const {
	Const init_data(State::Sx, width * size);
	for (auto &init : inits) {
		int offset = (init.addr.as_int() - start_offset) * width;
		for (int i = 0; i < GetSize(init.data); i++)
			if (0 <= i+offset && i+offset < GetSize(init_data))
				init_data.bits[i+offset] = init.data.bits[i];
	}
	return init_data;
}

namespace {

	struct MemIndex {
		dict<IdString, pool<Cell *>> rd_ports;
		dict<IdString, pool<Cell *>> wr_ports;
		dict<IdString, pool<Cell *>> inits;
		MemIndex (Module *module) {
			for (auto cell: module->cells()) {
				if (cell->type == ID($memwr))
					wr_ports[cell->parameters.at(ID::MEMID).decode_string()].insert(cell);
				else if (cell->type == ID($memrd))
					rd_ports[cell->parameters.at(ID::MEMID).decode_string()].insert(cell);
				else if (cell->type == ID($meminit))
					inits[cell->parameters.at(ID::MEMID).decode_string()].insert(cell);
			}
		}
	};

	Mem mem_from_memory(Module *module, RTLIL::Memory *mem, const MemIndex &index) {
		Mem res(module, mem->name, mem->width, mem->start_offset, mem->size);
		res.packed = false;
		res.mem = mem;
		res.attributes = mem->attributes;
		if (index.rd_ports.count(mem->name)) {
			for (auto cell : index.rd_ports.at(mem->name)) {
				MemRd mrd;
				mrd.cell = cell;
				mrd.attributes = cell->attributes;
				mrd.clk_enable = cell->parameters.at(ID::CLK_ENABLE).as_bool();
				mrd.clk_polarity = cell->parameters.at(ID::CLK_POLARITY).as_bool();
				mrd.ce_over_srst = cell->parameters.at(ID::CE_OVER_SRST).as_bool();
				mrd.arst_value = cell->parameters.at(ID::ARST_VALUE);
				mrd.srst_value = cell->parameters.at(ID::SRST_VALUE);
				mrd.init_value = cell->parameters.at(ID::INIT_VALUE);
				mrd.clk = cell->getPort(ID::CLK);
				mrd.en = cell->getPort(ID::EN);
				mrd.arst = cell->getPort(ID::ARST);
				mrd.srst = cell->getPort(ID::SRST);
				mrd.addr = cell->getPort(ID::ADDR);
				mrd.data = cell->getPort(ID::DATA);
				res.rd_ports.push_back(mrd);
			}
		}
		if (index.wr_ports.count(mem->name)) {
			std::vector<std::pair<int, MemWr>> ports;
			for (auto cell : index.wr_ports.at(mem->name)) {
				MemWr mwr;
				mwr.cell = cell;
				mwr.attributes = cell->attributes;
				mwr.clk_enable = cell->parameters.at(ID::CLK_ENABLE).as_bool();
				mwr.clk_polarity = cell->parameters.at(ID::CLK_POLARITY).as_bool();
				mwr.clk = cell->getPort(ID::CLK);
				mwr.en = cell->getPort(ID::EN);
				mwr.addr = cell->getPort(ID::ADDR);
				mwr.data = cell->getPort(ID::DATA);
				ports.push_back(std::make_pair(cell->parameters.at(ID::PORTID).as_int(), mwr));
			}
			std::sort(ports.begin(), ports.end(), [](const std::pair<int, MemWr> &a, const std::pair<int, MemWr> &b) { return a.first < b.first; });
			for (auto &it : ports)
				res.wr_ports.push_back(it.second);
			for (auto &port : res.wr_ports) {
				Const orig_prio_mask = port.cell->parameters.at(ID::PRIORITY_MASK);
				for (int i = 0; i < GetSize(res.wr_ports); i++) {
					auto &other_port = res.wr_ports[i];
					int other_portid = other_port.cell->parameters.at(ID::PORTID).as_int();
					bool has_prio = other_portid < GetSize(orig_prio_mask) && orig_prio_mask[other_portid] == State::S1;
					port.priority_mask.push_back(has_prio);
				}
			}
		}
		for (auto &port : res.rd_ports) {
			Const orig_trans_mask = port.cell->parameters.at(ID::TRANSPARENCY_MASK);
			for (int i = 0; i < GetSize(res.wr_ports); i++) {
				auto &other_port = res.wr_ports[i];
				int other_portid = other_port.cell->parameters.at(ID::PORTID).as_int();
				bool is_trans = other_portid < GetSize(orig_trans_mask) && orig_trans_mask[other_portid] == State::S1;
				port.transparency_mask.push_back(is_trans);
			}
		}
		if (index.inits.count(mem->name)) {
			std::vector<std::pair<int, MemInit>> inits;
			for (auto cell : index.inits.at(mem->name)) {
				MemInit init;
				init.cell = cell;
				init.attributes = cell->attributes;
				auto addr = cell->getPort(ID::ADDR);
				auto data = cell->getPort(ID::DATA);
				if (!addr.is_fully_const())
					log_error("Non-constant address %s in memory initialization %s.\n", log_signal(addr), log_id(cell));
				if (!data.is_fully_const())
					log_error("Non-constant data %s in memory initialization %s.\n", log_signal(data), log_id(cell));
				init.addr = addr.as_const();
				init.data = data.as_const();
				inits.push_back(std::make_pair(cell->parameters.at(ID::PRIORITY).as_int(), init));
			}
			std::sort(inits.begin(), inits.end(), [](const std::pair<int, MemInit> &a, const std::pair<int, MemInit> &b) { return a.first < b.first; });
			for (auto &it : inits)
				res.inits.push_back(it.second);
		}
		return res;
	}

	Mem mem_from_cell(Cell *cell) {
		Mem res(cell->module, cell->parameters.at(ID::MEMID).decode_string(),
			cell->parameters.at(ID::WIDTH).as_int(),
			cell->parameters.at(ID::OFFSET).as_int(),
			cell->parameters.at(ID::SIZE).as_int()
		);
		int abits = cell->parameters.at(ID::ABITS).as_int();
		res.packed = true;
		res.cell = cell;
		res.attributes = cell->attributes;
		Const &init = cell->parameters.at(ID::INIT);
		if (!init.is_fully_undef()) {
			int pos = 0;
			while (pos < res.size) {
				Const word = init.extract(pos * res.width, res.width, State::Sx);
				if (word.is_fully_undef()) {
					pos++;
				} else {
					int epos;
					for (epos = pos; epos < res.size; epos++) {
						Const eword = init.extract(epos * res.width, res.width, State::Sx);
						if (eword.is_fully_undef())
							break;
					}
					MemInit minit;
					minit.addr = res.start_offset + pos;
					minit.data = init.extract(pos * res.width, (epos - pos) * res.width, State::Sx);
					res.inits.push_back(minit);
					pos = epos;
				}
			}
		}
		int n_wr_ports = cell->parameters.at(ID::WR_PORTS).as_int();
		for (int i = 0; i < cell->parameters.at(ID::RD_PORTS).as_int(); i++) {
			MemRd mrd;
			mrd.clk_enable = cell->parameters.at(ID::RD_CLK_ENABLE).extract(i, 1).as_bool();
			mrd.clk_polarity = cell->parameters.at(ID::RD_CLK_POLARITY).extract(i, 1).as_bool();
			mrd.transparency_mask.resize(n_wr_ports);
			for (int j = 0; j < n_wr_ports; j++)
				mrd.transparency_mask[j] = cell->parameters.at(ID::RD_TRANSPARENCY_MASK).extract(i * n_wr_ports + j, 1).as_bool();
			mrd.ce_over_srst = cell->parameters.at(ID::RD_CE_OVER_SRST).extract(i, 1).as_bool();
			mrd.arst_value = cell->parameters.at(ID::RD_ARST_VALUE).extract(i * res.width, res.width);
			mrd.srst_value = cell->parameters.at(ID::RD_SRST_VALUE).extract(i * res.width, res.width);
			mrd.init_value = cell->parameters.at(ID::RD_INIT_VALUE).extract(i * res.width, res.width);
			mrd.clk = cell->getPort(ID::RD_CLK).extract(i, 1);
			mrd.en = cell->getPort(ID::RD_EN).extract(i, 1);
			mrd.arst = cell->getPort(ID::RD_ARST).extract(i, 1);
			mrd.srst = cell->getPort(ID::RD_SRST).extract(i, 1);
			mrd.addr = cell->getPort(ID::RD_ADDR).extract(i * abits, abits);
			mrd.data = cell->getPort(ID::RD_DATA).extract(i * res.width, res.width);
			res.rd_ports.push_back(mrd);
		}
		for (int i = 0; i < n_wr_ports; i++) {
			MemWr mwr;
			mwr.clk_enable = cell->parameters.at(ID::WR_CLK_ENABLE).extract(i, 1).as_bool();
			mwr.clk_polarity = cell->parameters.at(ID::WR_CLK_POLARITY).extract(i, 1).as_bool();
			mwr.priority_mask.resize(n_wr_ports);
			for (int j = 0; j < n_wr_ports; j++)
				mwr.priority_mask[j] = cell->parameters.at(ID::WR_PRIORITY_MASK).extract(i * n_wr_ports + j, 1).as_bool();
			mwr.clk = cell->getPort(ID::WR_CLK).extract(i, 1);
			mwr.en = cell->getPort(ID::WR_EN).extract(i * res.width, res.width);
			mwr.addr = cell->getPort(ID::WR_ADDR).extract(i * abits, abits);
			mwr.data = cell->getPort(ID::WR_DATA).extract(i * res.width, res.width);
			res.wr_ports.push_back(mwr);
		}
		return res;
	}

}

std::vector<Mem> Mem::get_all_memories(Module *module) {
	std::vector<Mem> res;
	MemIndex index(module);
	for (auto it: module->memories) {
		res.push_back(mem_from_memory(module, it.second, index));
	}
	for (auto cell: module->cells()) {
		if (cell->type == ID($mem))
			res.push_back(mem_from_cell(cell));
	}
	return res;
}

std::vector<Mem> Mem::get_selected_memories(Module *module) {
	std::vector<Mem> res;
	MemIndex index(module);
	for (auto it: module->memories) {
		if (module->design->selected(module, it.second))
			res.push_back(mem_from_memory(module, it.second, index));
	}
	for (auto cell: module->selected_cells()) {
		if (cell->type == ID($mem))
			res.push_back(mem_from_cell(cell));
	}
	return res;
}

Cell *Mem::extract_rdff(int idx, FfInitVals *initvals) {
	MemRd &port = rd_ports[idx];

	if (!port.clk_enable)
		return nullptr;

	Cell *c;

	SigSpec async_d = module->addWire(stringf("%s$rdreg[%d]$d", memid.c_str(), idx), width);
	SigSpec sig_d = async_d;

	for (int i = 0; i < GetSize(wr_ports); i++) {
		auto &wport = wr_ports[i];
		if (port.transparency_mask[i]) {
			log_assert(wport.clk_enable);
			log_assert(wport.clk == port.clk);
			log_assert(wport.clk_enable == port.clk_enable);
			SigSpec addr_eq;
			if (port.addr != wport.addr)
				addr_eq = module->Eq(stringf("%s$rdtransen[%d][%d]$d", memid.c_str(), idx, i), port.addr, wport.addr);
			int pos = 0;
			while (pos < width) {
				int epos = pos;
				while (epos < width && wport.en[epos] == wport.en[pos])
					epos++;
				SigSpec cur = sig_d.extract(pos, epos-pos);
				SigSpec other = wport.data.extract(pos, epos-pos);
				SigSpec cond;
				if (port.addr != wport.addr)
					cond = module->And(stringf("%s$rdtransgate[%d][%d][%d]$d", memid.c_str(), idx, i, pos), wport.en[pos], addr_eq);
				else
					cond = wport.en[pos];
				SigSpec merged = module->Mux(stringf("%s$rdtransmux[%d][%d][%d]$d", memid.c_str(), idx, i, pos), cur, other, cond);
				sig_d.replace(pos, merged);
				pos = epos;
			}
		}
	}

	log_assert(port.arst == State::S0 || port.srst == State::S0);
	IdString name = stringf("%s$rdreg[%d]", memid.c_str(), idx);
	FfData ff(initvals);
	ff.width = width;
	ff.has_clk = true;
	ff.sig_clk = port.clk;
	ff.pol_clk = port.clk_polarity;
	if (port.en != State::S1) {
		ff.has_en = true;
		ff.pol_en = true;
		ff.sig_en = port.en;
	}
	if (port.arst != State::S0) {
		ff.has_arst = true;
		ff.sig_arst = port.arst;
		ff.pol_arst = true;
		ff.val_arst = port.arst_value;
	}
	if (port.srst != State::S0) {
		ff.has_srst = true;
		ff.sig_srst = port.srst;
		ff.pol_srst = true;
		ff.val_srst = port.srst_value;
		ff.ce_over_srst = ff.has_en && port.ce_over_srst;
	}
	ff.sig_d = sig_d;
	ff.sig_q = port.data;
	ff.val_init = port.init_value;
	port.data = async_d;
	c = ff.emit(module, name);

	log("Extracted data FF from read port %d of %s.%s: %s\n",
			idx, log_id(module), log_id(memid), log_id(c));

	port.en = State::S1;
	port.clk = State::S0;
	port.arst = State::S0;
	port.srst = State::S0;
	port.clk_enable = false;
	port.clk_polarity = true;
	port.ce_over_srst = false;
	port.arst_value = Const(State::Sx, width);
	port.srst_value = Const(State::Sx, width);
	port.init_value = Const(State::Sx, width);

	return c;
}
