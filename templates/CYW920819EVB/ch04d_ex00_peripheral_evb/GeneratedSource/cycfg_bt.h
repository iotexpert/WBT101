/***************************************************************************//**
* File Name: cycfg_bt.h
* Version: 1.1
*
* Description:
* Bluetooth Configurator configuration.
* This file should not be modified. It was automatically generated by
* Bluetooth Configurator 1.1.0 build 291
*
********************************************************************************
* Copyright 2019 Cypress Semiconductor Corporation
* SPDX-License-Identifier: Apache-2.0
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

#if !defined(CYCFG_BT_H)
#define CYCFG_BT_H

/*
BLE_CONFIG_START
<?xml version="1.0"?>
<Configuration major="1" minor="1" device="20xxx">
    <Profiles>
        <Profile name="GATT">
            <ProfileRoles>
                <ProfileRole type="Server">
                    <ProfileRoleProperties>
                        <Property id="Name" value="Server"/>
                    </ProfileRoleProperties>
                    <Services>
                        <Service type="org.bluetooth.service.generic_access">
                            <ServiceProperties>
                                <Property id="EntityID" value="{53539bbd-6f53-4ff7-92f7-6a6aaf483d9f}"/>
                                <Property id="ServiceDeclaration" value="Primary"/>
                            </ServiceProperties>
                            <Characteristics>
                                <Characteristic type="org.bluetooth.characteristic.gap.device_name">
                                    <Fields>
                                        <Field>
                                            <FieldProperties>
                                                <Property id="Name" value="Name"/>
                                                <Property id="Value" value="change_me"/>
                                                <Property id="Format" value="f_utf8s"/>
                                                <Property id="ByteLength" value="9"/>
                                            </FieldProperties>
                                        </Field>
                                    </Fields>
                                    <Properties>
                                        <BleProperty>
                                            <Property id="PropertyType" value="Read"/>
                                            <Property id="Present" value="true"/>
                                            <Property id="Mandatory" value="true"/>
                                        </BleProperty>
                                        <BleProperty>
                                            <Property id="PropertyType" value="Write"/>
                                            <Property id="Present" value="false"/>
                                            <Property id="Mandatory" value="false"/>
                                        </BleProperty>
                                    </Properties>
                                    <Permission>
                                        <Property id="Read" value="true"/>
                                        <Property id="ReadAuthenticated" value="false"/>
                                        <Property id="VariableLength" value="false"/>
                                        <Property id="Write" value="false"/>
                                        <Property id="WriteNoResponse" value="false"/>
                                        <Property id="WriteAuthenticated" value="false"/>
                                        <Property id="WriteReliable" value="false"/>
                                    </Permission>
                                    <Descriptors/>
                                </Characteristic>
                                <Characteristic type="org.bluetooth.characteristic.gap.appearance">
                                    <Fields>
                                        <Field>
                                            <FieldProperties>
                                                <Property id="Name" value="Category"/>
                                                <Property id="EnumValue" value="0"/>
                                                <Property id="Format" value="f_16bit"/>
                                            </FieldProperties>
                                        </Field>
                                    </Fields>
                                    <Properties>
                                        <BleProperty>
                                            <Property id="PropertyType" value="Read"/>
                                            <Property id="Present" value="true"/>
                                            <Property id="Mandatory" value="true"/>
                                        </BleProperty>
                                    </Properties>
                                    <Permission>
                                        <Property id="Read" value="true"/>
                                        <Property id="ReadAuthenticated" value="false"/>
                                        <Property id="VariableLength" value="false"/>
                                        <Property id="Write" value="false"/>
                                        <Property id="WriteNoResponse" value="false"/>
                                        <Property id="WriteAuthenticated" value="false"/>
                                        <Property id="WriteReliable" value="false"/>
                                    </Permission>
                                    <Descriptors/>
                                </Characteristic>
                            </Characteristics>
                        </Service>
                        <Service type="org.bluetooth.service.generic_attribute">
                            <ServiceProperties>
                                <Property id="EntityID" value="{0570e760-4e46-49ab-96eb-7e0c71dba816}"/>
                                <Property id="ServiceDeclaration" value="Primary"/>
                            </ServiceProperties>
                            <Characteristics/>
                        </Service>
                        <Service type="org.bluetooth.service.custom">
                            <ServiceProperties>
                                <Property id="EntityID" value="{b168a47b-1eda-4b32-8fc3-af7c277ee406}"/>
                                <Property id="DisplayName" value="Modus"/>
                                <Property id="UUID" value="7991826E-0776-47EC-8D44-6827CA06710A"/>
                                <Property id="ServiceDeclaration" value="Primary"/>
                            </ServiceProperties>
                            <Characteristics>
                                <Characteristic type="org.bluetooth.characteristic.custom">
                                    <CharacteristicProperties>
                                        <Property id="DisplayName" value="LED"/>
                                        <Property id="UUID" value="02AA0595-819D-4674-99AD-7D383CB96674"/>
                                    </CharacteristicProperties>
                                    <Fields>
                                        <Field>
                                            <FieldProperties>
                                                <Property id="Name" value="New field"/>
                                                <Property id="Value" value="0"/>
                                                <Property id="Format" value="f_uint8"/>
                                            </FieldProperties>
                                        </Field>
                                    </Fields>
                                    <Properties>
                                        <BleProperty>
                                            <Property id="PropertyType" value="Read"/>
                                            <Property id="Present" value="true"/>
                                            <Property id="Mandatory" value="false"/>
                                        </BleProperty>
                                        <BleProperty>
                                            <Property id="PropertyType" value="Write"/>
                                            <Property id="Present" value="true"/>
                                            <Property id="Mandatory" value="false"/>
                                        </BleProperty>
                                        <BleProperty>
                                            <Property id="PropertyType" value="WriteWithoutResponse"/>
                                            <Property id="Present" value="false"/>
                                            <Property id="Mandatory" value="false"/>
                                        </BleProperty>
                                        <BleProperty>
                                            <Property id="PropertyType" value="AuthenticatedSignedWrites"/>
                                            <Property id="Present" value="false"/>
                                            <Property id="Mandatory" value="false"/>
                                        </BleProperty>
                                        <BleProperty>
                                            <Property id="PropertyType" value="ReliableWrite"/>
                                            <Property id="Present" value="false"/>
                                            <Property id="Mandatory" value="false"/>
                                        </BleProperty>
                                        <BleProperty>
                                            <Property id="PropertyType" value="Notify"/>
                                            <Property id="Present" value="false"/>
                                            <Property id="Mandatory" value="false"/>
                                        </BleProperty>
                                        <BleProperty>
                                            <Property id="PropertyType" value="Indicate"/>
                                            <Property id="Present" value="false"/>
                                            <Property id="Mandatory" value="false"/>
                                        </BleProperty>
                                        <BleProperty>
                                            <Property id="PropertyType" value="WritableAuxiliaries"/>
                                            <Property id="Present" value="false"/>
                                            <Property id="Mandatory" value="false"/>
                                        </BleProperty>
                                        <BleProperty>
                                            <Property id="PropertyType" value="Broadcast"/>
                                            <Property id="Present" value="false"/>
                                            <Property id="Mandatory" value="false"/>
                                        </BleProperty>
                                    </Properties>
                                    <Permission>
                                        <Property id="Read" value="true"/>
                                        <Property id="ReadAuthenticated" value="false"/>
                                        <Property id="VariableLength" value="false"/>
                                        <Property id="Write" value="true"/>
                                        <Property id="WriteNoResponse" value="false"/>
                                        <Property id="WriteAuthenticated" value="false"/>
                                        <Property id="WriteReliable" value="false"/>
                                    </Permission>
                                    <Descriptors/>
                                </Characteristic>
                                <Characteristic type="org.bluetooth.characteristic.custom">
                                    <CharacteristicProperties>
                                        <Property id="DisplayName" value="Counter"/>
                                        <Property id="UUID" value="FCA5B263-0A69-4589-967B-9FEB13ECB968"/>
                                    </CharacteristicProperties>
                                    <Fields>
                                        <Field>
                                            <FieldProperties>
                                                <Property id="Name" value="New field"/>
                                                <Property id="Value" value="0"/>
                                                <Property id="Format" value="f_uint8"/>
                                            </FieldProperties>
                                        </Field>
                                    </Fields>
                                    <Properties>
                                        <BleProperty>
                                            <Property id="PropertyType" value="Read"/>
                                            <Property id="Present" value="true"/>
                                            <Property id="Mandatory" value="false"/>
                                        </BleProperty>
                                        <BleProperty>
                                            <Property id="PropertyType" value="Write"/>
                                            <Property id="Present" value="false"/>
                                            <Property id="Mandatory" value="false"/>
                                        </BleProperty>
                                        <BleProperty>
                                            <Property id="PropertyType" value="WriteWithoutResponse"/>
                                            <Property id="Present" value="false"/>
                                            <Property id="Mandatory" value="false"/>
                                        </BleProperty>
                                        <BleProperty>
                                            <Property id="PropertyType" value="AuthenticatedSignedWrites"/>
                                            <Property id="Present" value="false"/>
                                            <Property id="Mandatory" value="false"/>
                                        </BleProperty>
                                        <BleProperty>
                                            <Property id="PropertyType" value="ReliableWrite"/>
                                            <Property id="Present" value="false"/>
                                            <Property id="Mandatory" value="false"/>
                                        </BleProperty>
                                        <BleProperty>
                                            <Property id="PropertyType" value="Notify"/>
                                            <Property id="Present" value="true"/>
                                            <Property id="Mandatory" value="false"/>
                                        </BleProperty>
                                        <BleProperty>
                                            <Property id="PropertyType" value="Indicate"/>
                                            <Property id="Present" value="false"/>
                                            <Property id="Mandatory" value="false"/>
                                        </BleProperty>
                                        <BleProperty>
                                            <Property id="PropertyType" value="WritableAuxiliaries"/>
                                            <Property id="Present" value="false"/>
                                            <Property id="Mandatory" value="false"/>
                                        </BleProperty>
                                        <BleProperty>
                                            <Property id="PropertyType" value="Broadcast"/>
                                            <Property id="Present" value="false"/>
                                            <Property id="Mandatory" value="false"/>
                                        </BleProperty>
                                    </Properties>
                                    <Permission>
                                        <Property id="Read" value="true"/>
                                        <Property id="ReadAuthenticated" value="true"/>
                                        <Property id="VariableLength" value="false"/>
                                        <Property id="Write" value="false"/>
                                        <Property id="WriteNoResponse" value="false"/>
                                        <Property id="WriteAuthenticated" value="false"/>
                                        <Property id="WriteReliable" value="false"/>
                                    </Permission>
                                    <Descriptors>
                                        <Descriptor type="org.bluetooth.descriptor.gatt.client_characteristic_configuration">
                                            <Fields>
                                                <Field>
                                                    <FieldProperties>
                                                        <Property id="Name" value="Properties"/>
                                                        <Property id="Value" value=""/>
                                                        <Property id="Format" value="f_16bit"/>
                                                    </FieldProperties>
                                                    <BitField>
                                                        <Property id="BitValue" value="0"/>
                                                        <Property id="BitValue" value="0"/>
                                                    </BitField>
                                                </Field>
                                            </Fields>
                                            <Properties>
                                                <BleProperty>
                                                    <Property id="PropertyType" value="Read"/>
                                                    <Property id="Present" value="true"/>
                                                    <Property id="Mandatory" value="false"/>
                                                </BleProperty>
                                                <BleProperty>
                                                    <Property id="PropertyType" value="Write"/>
                                                    <Property id="Present" value="true"/>
                                                    <Property id="Mandatory" value="false"/>
                                                </BleProperty>
                                            </Properties>
                                            <Permission>
                                                <Property id="Read" value="true"/>
                                                <Property id="ReadAuthenticated" value="false"/>
                                                <Property id="VariableLength" value="false"/>
                                                <Property id="Write" value="true"/>
                                                <Property id="WriteNoResponse" value="false"/>
                                                <Property id="WriteAuthenticated" value="true"/>
                                                <Property id="WriteReliable" value="false"/>
                                            </Permission>
                                        </Descriptor>
                                    </Descriptors>
                                </Characteristic>
                            </Characteristics>
                        </Service>
                    </Services>
                </ProfileRole>
            </ProfileRoles>
        </Profile>
    </Profiles>
</Configuration>
BLE_CONFIG_END
*/

#endif /* CYCFG_BT_H */

/* [] END OF FILE */

